"""KiCad IPC API parser.

Reads the board from a running KiCad instance through the IPC API using
kicad-python (kipy). This replaces the legacy SWIG ``pcbnew`` parser:
the ``pcbnew`` Python module is deprecated and is removed in KiCad 11.

The parser produces the same pcbdata structure as the old PcbnewParser,
so the html renderer and the rest of iBOM are unchanged.

Requirements: KiCad 9.0.3+ with the API server enabled
(Preferences > Plugins > Enable KiCad API) and kicad-python 0.5+.
Footprint variants need a KiCad / kicad-python version that exposes
``FootprintInstance.variants`` (KiCad 10.0.7+).
"""

import math
import os
from datetime import datetime

from kipy import KiCad
from kipy.board_types import (
    ArcTrack,
    BoardArc,
    BoardBezier,
    BoardCircle,
    BoardPolygon,
    BoardRectangle,
    BoardSegment,
    BoardText,
    BoardTextBox,
    Field,
    Via,
)
from kipy.common_types import Polygon, Segment, Text
from kipy.proto.board import board_types_pb2

from .common import EcadParser, Component, ExtraFieldData
from .kicad_extra import find_latest_schematic_data, parse_schematic_data
from ..errors import ParsingException

BL = board_types_pb2.BoardLayer
PAD_TYPE = board_types_pb2.PadType
PAD_STACK_TYPE = board_types_pb2.PadStackType
PAD_STACK_SHAPE = board_types_pb2.PadStackShape
DRILL_SHAPE = board_types_pb2.DrillShape
ULR = board_types_pb2.UnconnectedLayerRemoval
SOLDER_MASK_MODE = board_types_pb2.SolderMaskMode

OUTER_COPPER_LAYERS = [(BL.BL_F_Cu, "F"), (BL.BL_B_Cu, "B")]
# Inner copper layers in stack order; BoardLayer values follow it too
# (F_Cu < In1_Cu < ... < In30_Cu < B_Cu).
INNER_COPPER_LAYERS = [(getattr(BL, 'BL_In%d_Cu' % i), 'In%d.Cu' % i)
                       for i in range(1, 31)]
INNER_COPPER_LAYER_IDS = set(layer for layer, _ in INNER_COPPER_LAYERS)

SHAPE_TYPES = (BoardSegment, BoardRectangle, BoardCircle, BoardArc,
               BoardPolygon, BoardBezier)
TEXT_TYPES = (BoardText, BoardTextBox, Field)

# Bitmask values matching pcbnew's RECT_CHAMFER_POSITIONS, which is what
# the html renderer expects in the "chamfpos" field.
CHAMFER_TOP_LEFT = 1
CHAMFER_TOP_RIGHT = 2
CHAMFER_BOTTOM_LEFT = 4
CHAMFER_BOTTOM_RIGHT = 8

# Maximum deviation (in nm) when approximating arcs inside polygon
# outlines with line segments. Same value the old parser used for drills.
ARC_APPROXIMATION_ERROR = 10000


def create_path(lines):
    """Returns an svg path d-string for a list of polylines."""

    def limit_digits(val):
        return format(val, '.6f').rstrip('0').replace(',', '.').rstrip('.')

    def different_points(a, b):
        return abs(a[0] - b[0]) > 1e-6 or abs(a[1] - b[1]) > 1e-6

    parts = []
    for i, line in enumerate(lines):
        if i == 0 or different_points(lines[i - 1][-1], line[0]):
            parts.append('M{},{}'.format(*map(limit_digits, line[0])))
        for point in line[1:]:
            parts.append('L{},{}'.format(*map(limit_digits, point)))
    return ''.join(parts)


def board_file_path(board):
    """Absolute path of the .kicad_pcb file of a kipy Board, or ''."""
    name = board.name
    if not name:
        return ''
    if os.path.isabs(name):
        return name
    return os.path.join(board.get_project().path, name)


def connect_to_kicad(file_name=None):
    """Connects to the running KiCad instance.

    Connection parameters come from KICAD_API_SOCKET / KICAD_API_TOKEN
    environment variables when launched by KiCad as a plugin, otherwise the
    default per-user socket is used.

    :param file_name: optional .kicad_pcb path; if given it must be the
                      board that is currently open in KiCad.
    :return: (kicad, board, pcb_file_name)
    :raises ParsingException: when KiCad or the board is not available.
    """
    try:
        kicad = KiCad()
        board = kicad.get_board()
    except Exception as e:
        raise ParsingException(
            'Could not get the board from KiCad over the IPC API (%s).\n'
            'Make sure KiCad is running with the board open in the PCB '
            'editor and that the API server is enabled in '
            'Preferences > Plugins.' % e)

    pcb_file_name = board_file_path(board)
    if not pcb_file_name:
        raise ParsingException(
            'Please save the board file before generating BOM.')

    if file_name:
        def norm(p):
            return os.path.normcase(os.path.realpath(os.path.abspath(p)))
        if norm(file_name) != norm(pcb_file_name):
            raise ParsingException(
                '%s is not the board open in KiCad (%s).\n'
                'KiCad boards are read over the IPC API, so the board must '
                'be open in the PCB editor of a running KiCad.'
                % (file_name, pcb_file_name))

    return kicad, board, pcb_file_name


class KiCadIPCParser(EcadParser):

    def __init__(self, file_name, config, logger, kicad=None, board=None):
        if kicad is None:
            kicad = KiCad() if board is None else KiCad.from_client(
                board.client)
        self.kicad = kicad
        self.board = board if board is not None else kicad.get_board()
        if not file_name:
            file_name = board_file_path(self.board)
        super(KiCadIPCParser, self).__init__(file_name, config, logger)
        self.footprints = list(self.board.get_footprints())
        self._text_render_cache = {}
        self._reported = set()

    # ------------------------------------------------------------------
    # helpers

    def _report_once(self, key, msg):
        if key not in self._reported:
            self._reported.add(key)
            self.logger.info(msg)

    @staticmethod
    def normalize(point):
        return [point.x * 1e-6, point.y * 1e-6]

    @staticmethod
    def rotate(point, center, angle_degrees):
        # Rotation in kicad's coordinate system (y axis down,
        # positive angle is counterclockwise on screen).
        a = -math.radians(angle_degrees)
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        return [center[0] + dx * math.cos(a) - dy * math.sin(a),
                center[1] + dx * math.sin(a) + dy * math.cos(a)]

    @staticmethod
    def layer_letter(layer):
        return {BL.BL_F_Cu: "F", BL.BL_B_Cu: "B"}.get(layer)

    @staticmethod
    def _arc_sweep(a1, am, a2):
        """Signed sweep (radians) from a1 to a2 passing through am."""
        two_pi = 2 * math.pi
        ccw_mid = (am - a1) % two_pi
        ccw_end = (a2 - a1) % two_pi
        if ccw_mid <= ccw_end:
            return ccw_end
        return ccw_end - two_pi

    def approximate_arc(self, start, mid, end):
        """Approximates a start/mid/end arc with points (in mm)."""
        sx, sy, mx, my, ex, ey = (start.x, start.y, mid.x, mid.y,
                                  end.x, end.y)
        d = 2 * (sx * (my - ey) + mx * (ey - sy) + ex * (sy - my))
        if abs(d) < 1e-9:
            # Degenerate (collinear) arc.
            return [self.normalize(start), self.normalize(end)]
        s2, m2, e2 = sx * sx + sy * sy, mx * mx + my * my, ex * ex + ey * ey
        cx = (s2 * (my - ey) + m2 * (ey - sy) + e2 * (sy - my)) / d
        cy = (s2 * (ex - mx) + m2 * (sx - ex) + e2 * (mx - sx)) / d
        r = math.hypot(sx - cx, sy - cy)
        a1 = math.atan2(sy - cy, sx - cx)
        am = math.atan2(my - cy, mx - cx)
        a2 = math.atan2(ey - cy, ex - cx)
        sweep = self._arc_sweep(a1, am, a2)
        if r > ARC_APPROXIMATION_ERROR:
            step = 2 * math.acos(1 - ARC_APPROXIMATION_ERROR / r)
            n = max(2, int(math.ceil(abs(sweep) / step)))
        else:
            n = 2
        points = []
        for i in range(n + 1):
            a = a1 + sweep * i / n
            points.append([(cx + r * math.cos(a)) * 1e-6,
                           (cy + r * math.sin(a)) * 1e-6])
        return points

    def parse_poly_line(self, poly_line):
        # type: (kipy.geometry.PolyLine) -> list
        result = []
        for node in poly_line.nodes:
            if node.has_point:
                result.append(self.normalize(node.point))
            elif node.has_arc:
                arc = node.arc
                points = self.approximate_arc(arc.start, arc.mid, arc.end)
                if result and result[-1] == points[0]:
                    points = points[1:]
                result.extend(points)
        if len(result) > 1 and result[0] == result[-1]:
            # Closing point is implicit in the data format.
            result.pop()
        return result

    def parse_polygons(self, polygons_with_holes):
        """Converts kipy PolygonWithHoles list into a list of contours.

        Holes are emitted as additional contours right after their
        outline, same as the old parser did for SHAPE_POLY_SET.
        """
        result = []
        for poly in polygons_with_holes:
            result.append(self.parse_poly_line(poly.outline))
            for hole in poly.holes:
                result.append(self.parse_poly_line(hole))
        return [c for c in result if c]

    @staticmethod
    def _has_holes(polygons_with_holes):
        return any(len(p.holes) > 0 for p in polygons_with_holes)

    def arc_angles_degrees(self, shape):
        """Computes (startangle, endangle) like the old parser did.

        Angles are in kicad's screen coordinate system (y axis down,
        angles grow clockwise on screen). They are computed from the
        arc's start/mid/end points directly since kipy's angle helpers
        use a different convention.
        """
        center = shape.center()
        if center is None:
            return 0, 0

        def angle_to(p):
            return math.atan2(p.y - center.y, p.x - center.x)

        a1 = angle_to(shape.start)
        sweep = math.degrees(
            self._arc_sweep(a1, angle_to(shape.mid), angle_to(shape.end)))
        a1 = math.degrees(a1)
        if a1 >= 180 - 1e-3:
            a1 -= 360  # match pcbnew's [-180, 180) range
        a2 = a1 + sweep
        if a2 < a1:
            a1, a2 = a2, a1
        return round(a1, 2), round(a2, 2)

    # ------------------------------------------------------------------
    # graphic shapes

    def parse_shape(self, d):
        width = d.attributes.stroke.width * 1e-6
        if isinstance(d, BoardSegment):
            return {
                "type": "segment",
                "start": self.normalize(d.start),
                "end": self.normalize(d.end),
                "width": width,
            }
        if isinstance(d, BoardRectangle):
            start = self.normalize(d.top_left)
            end = self.normalize(d.bottom_right)
            return {
                "type": "polygon",
                "pos": [0, 0],
                "angle": 0,
                "polygons": [[
                    start,
                    [end[0], start[1]],
                    end,
                    [start[0], end[1]],
                ]],
                "width": width,
                "filled": 1 if d.attributes.fill.filled else 0,
            }
        if isinstance(d, BoardCircle):
            radius = math.hypot(d.radius_point.x - d.center.x,
                                d.radius_point.y - d.center.y)
            shape_dict = {
                "type": "circle",
                "start": self.normalize(d.center),
                "radius": radius * 1e-6,
                "width": width,
            }
            if d.attributes.fill.filled:
                shape_dict["filled"] = 1
            return shape_dict
        if isinstance(d, BoardArc):
            center = d.center()
            if center is None:
                # Degenerate arc, draw it as a segment.
                return {
                    "type": "segment",
                    "start": self.normalize(d.start),
                    "end": self.normalize(d.end),
                    "width": width,
                }
            a1, a2 = self.arc_angles_degrees(d)
            return {
                "type": "arc",
                "start": self.normalize(center),
                "radius": math.hypot(d.start.x - center.x,
                                     d.start.y - center.y) * 1e-6,
                "startangle": a1,
                "endangle": a2,
                "width": width,
            }
        if isinstance(d, BoardPolygon):
            shape_dict = {
                "type": "polygon",
                "pos": [0, 0],
                "angle": 0,
                "polygons": self.parse_polygons(d.polygons),
            }
            if not d.attributes.fill.filled:
                shape_dict["filled"] = 0
                shape_dict["width"] = width
            return shape_dict
        if isinstance(d, BoardBezier):
            return {
                "type": "curve",
                "start": self.normalize(d.start),
                "cpa": self.normalize(d.control1),
                "cpb": self.normalize(d.control2),
                "end": self.normalize(d.end),
                "width": width,
            }
        self.logger.info("Unsupported shape %s, skipping", type(d).__name__)
        return None

    # ------------------------------------------------------------------
    # text

    def _footprint_value(self, footprint):
        variant = self._footprint_variant(footprint)
        if variant is not None:
            for k, v in variant.fields.items():
                if k.lower() == 'value' and v:
                    return v
        return footprint.value_field.text.value

    def _resolve_text_value(self, value, footprint):
        """Expands text variables. Footprint-context variables are
        substituted locally, the rest is expanded by KiCad."""
        if '${' not in value:
            return value
        if footprint is not None:
            subs = {
                'REFERENCE': footprint.reference_field.text.value,
                'VALUE': self._footprint_value(footprint),
                'FOOTPRINT_NAME': footprint.definition.id.name,
                'FOOTPRINT_LIBRARY': footprint.definition.id.library,
            }
            for key, sub in subs.items():
                value = value.replace('${%s}' % key, sub)
        if '${' in value:
            value = self.board.expand_text_variables(value)
        return value

    def _as_common_text(self, d, footprint=None):
        if isinstance(d, Field):
            text = d.text.as_text()
        elif isinstance(d, BoardText):
            text = d.as_text()
        elif isinstance(d, BoardTextBox):
            return d.as_textbox()
        else:
            text = d  # already a common Text

        modified = False

        # GetTextAsShapes renders the stored string as-is, with text
        # variables like ${REFERENCE} unexpanded. Resolve them first.
        value = self._resolve_text_value(text.value, footprint)
        if value != text.value:
            text = Text(text.proto)  # copy, never mutate board items
            text.value = value
            modified = True

        # GetTextAsShapes renders the stored angle as-is, but KiCad draws
        # keep-upright text rotated back into readable orientation
        # (see EDA_TEXT::GetDrawRotation). Replicate that here, otherwise
        # justified text on rotated footprints renders on the wrong side
        # of its anchor.
        attrs = text.proto.attributes
        if attrs.keep_upright:
            angle = attrs.angle.value_degrees % 360
            if 90 < angle <= 270:
                if not modified:
                    text = Text(text.proto)
                text.proto.attributes.angle.value_degrees = angle - 180
        return text

    def prerender_text_items(self, items):
        """Renders all text items in one IPC round trip.

        KiCad renders text to strokes / polygons server side, which
        replaces the SWIG GetEffectiveTextShape() call. Results are cached
        keyed by the text item's python object id.

        :param items: list of (text_item, parent_footprint_or_None)
        """
        items = [(t, f) for (t, f) in items
                 if id(t) not in self._text_render_cache]
        if not items:
            return
        shapes = self.kicad.get_text_as_shapes(
            [self._as_common_text(t, f) for (t, f) in items])
        for (item, _), compound in zip(items, shapes):
            self._text_render_cache[id(item)] = compound

    def parse_text(self, d, footprint=None):
        knockout_item = d.text if isinstance(d, Field) else d
        if getattr(knockout_item, 'knockout', False):
            self._report_once(
                'knockout',
                'Knockout text is not supported over the IPC API yet, '
                'it is rendered as regular text.')
        common_text = self._as_common_text(d, footprint)
        thickness = common_text.attributes.stroke_width * 1e-6
        compound = self._text_render_cache.get(id(d))
        if compound is None:
            compound = self.kicad.get_text_as_shapes(common_text)[0]
        segments = []
        polygons = []
        for s in compound.shapes:
            if isinstance(s, Segment):
                segments.append(
                    [self.normalize(s.start), self.normalize(s.end)])
            elif isinstance(s, Polygon):
                polygons.extend(self.parse_polygons(s.polygons))
            else:
                self.logger.warn(
                    "Unsupported subshape in text: %s" % type(s).__name__)
        if segments:
            return {
                "thickness": thickness,
                "svgpath": create_path(segments)
            }
        else:
            return {
                "polygons": polygons
            }

    def parse_drawing(self, d, footprint=None):
        if isinstance(d, SHAPE_TYPES):
            s = self.parse_shape(d)
            return [s] if s else []
        if isinstance(d, TEXT_TYPES):
            s = self.parse_text(d, footprint)
            return [s] if s else []
        self.logger.info("Unsupported drawing type %s, skipping",
                         type(d).__name__)
        return []

    def get_all_drawings(self):
        """Returns a list of (kind, item, parent_footprint) tuples for all
        board and footprint drawings.

        kind is "ref"/"val" for reference and value fields so they can be
        tagged in the output.
        """
        drawings = []
        for d in self.board.get_shapes():
            drawings.append(("shape", d, None))
        for t in self.board.get_text():
            drawings.append(("text", t, None))
        for f in self.footprints:
            # Note: BoardText.attributes.visible is deprecated and always
            # True over the IPC API; Field.visible is the reliable flag.
            if f.reference_field.visible:
                drawings.append(("ref", f.reference_field, f))
            if f.value_field.visible:
                drawings.append(("val", f.value_field, f))
            for d in f.definition.shapes:
                drawings.append(("shape", d, f))
            ref_val_ids = (f.reference_field.field_id,
                           f.value_field.field_id)
            for t in f.texts_and_fields:
                if isinstance(t, Field):
                    if t.field_id in ref_val_ids or not t.visible:
                        continue
                drawings.append(("text", t, f))
        return drawings

    def parse_drawings_on_layers(self, drawings, f_layer, b_layer):
        front = []
        back = []

        self.prerender_text_items(
            [(d, f) for (_, d, f) in drawings
             if d.layer in [f_layer, b_layer] and
             isinstance(d, TEXT_TYPES)])

        for kind, d, f in drawings:
            if d.layer not in [f_layer, b_layer]:
                continue
            for drawing in self.parse_drawing(d, f):
                if kind in ["ref", "val"]:
                    drawing[kind] = 1
                if d.layer == f_layer:
                    front.append(drawing)
                else:
                    back.append(drawing)

        return {
            "F": front,
            "B": back,
        }

    def report_unsupported_items(self):
        """Logs board items the IPC API can not provide geometry for."""
        layers = {BL.BL_F_SilkS, BL.BL_B_SilkS, BL.BL_F_Fab, BL.BL_B_Fab,
                  BL.BL_Edge_Cuts}
        for getter, what in [('get_dimensions', 'dimensions'),
                             ('get_barcodes', 'barcodes')]:
            fn = getattr(self.board, getter, None)
            if fn is None:
                continue
            try:
                count = sum(1 for i in fn()
                            if getattr(i, 'layer', None) in layers)
            except Exception:
                continue
            if count:
                self.logger.info(
                    'Skipped %d %s: their shapes are not available over '
                    'the KiCad IPC API yet.' % (count, what))

    # ------------------------------------------------------------------
    # edges / bounding boxes

    def _shape_points_with_extents(self, d):
        """Returns points describing the extents of a shape including
        its stroke width, used for bounding box computation."""
        width = d.attributes.stroke.width
        points = []
        if isinstance(d, BoardSegment):
            points = [d.start, d.end]
        elif isinstance(d, BoardRectangle):
            points = [d.top_left, d.bottom_right]
        elif isinstance(d, (BoardCircle, BoardArc, BoardPolygon,
                            BoardBezier)):
            try:
                box = d.bounding_box()
            except Exception:
                return []
            if box is None:
                return []
            points = [box.pos,
                      type(box.pos).from_xy(box.pos.x + box.size.x,
                                            box.pos.y + box.size.y)]
        result = []
        for p in points:
            x, y = p.x, p.y
            result.append([(x - width / 2) * 1e-6, (y - width / 2) * 1e-6])
            result.append([(x + width / 2) * 1e-6, (y + width / 2) * 1e-6])
        return result

    def parse_edges(self):
        edges = []
        bbox = None  # [minx, miny, maxx, maxy] in mm

        items = list(self.board.get_shapes())
        for f in self.footprints:
            items.extend(f.definition.shapes)

        for d in items:
            if d.layer != BL.BL_Edge_Cuts:
                continue
            for parsed_drawing in self.parse_drawing(d):
                edges.append(parsed_drawing)
            for p in self._shape_points_with_extents(d):
                if bbox is None:
                    bbox = [p[0], p[1], p[0], p[1]]
                else:
                    bbox = [min(bbox[0], p[0]), min(bbox[1], p[1]),
                            max(bbox[2], p[0]), max(bbox[3], p[1])]
        return edges, bbox

    # ------------------------------------------------------------------
    # footprints and pads

    @staticmethod
    def _pad_is_through_hole(pad):
        return pad.pad_type in [PAD_TYPE.PT_PTH, PAD_TYPE.PT_NPTH]

    def parse_pad(self, pad):
        padstack = pad.padstack
        layers_set = list(padstack.layers)
        layers = []
        for layer, letter in OUTER_COPPER_LAYERS:
            if layer in layers_set:
                layers.append(letter)
        if not layers and not self._pad_is_through_hole(pad):
            return []

        custom_padstack = (
            padstack.type != PAD_STACK_TYPE.PST_NORMAL or
            padstack.unconnected_layer_removal not in
            [ULR.ULR_KEEP, ULR.ULR_UNKNOWN]
        )

        if custom_padstack:
            presence = self.board.check_padstack_presence_on_layers(
                pad, [layer for layer, _ in OUTER_COPPER_LAYERS])
            presence = presence.get(pad, {})
            pads = []
            for layer, letter in OUTER_COPPER_LAYERS:
                if layer in layers_set and presence.get(layer, True):
                    pad_dict = self.parse_pad_layer(pad, layer)
                    if pad_dict is not None:
                        pad_dict["layers"] = [letter]
                        pads.append(pad_dict)
            return pads
        else:
            pad_layer = layers_set[0] if layers_set else BL.BL_F_Cu
            pad_dict = self.parse_pad_layer(pad, pad_layer)
            if pad_dict is None:
                return []
            pad_dict["layers"] = layers
            return [pad_dict]

    @staticmethod
    def _padstack_copper_layer(padstack, layer):
        """PadStackLayer describing the given copper layer, or None."""
        psl = padstack.copper_layer(layer)
        if psl is None and layer in INNER_COPPER_LAYER_IDS and \
                padstack.type == PAD_STACK_TYPE.PST_FRONT_INNER_BACK:
            # In front/inner/back mode In1_Cu describes all inner layers.
            psl = padstack.copper_layer(BL.BL_In1_Cu)
        if psl is None and padstack.copper_layers:
            psl = padstack.copper_layers[0]
        return psl

    def parse_pad_layer(self, pad, layer):
        padstack = pad.padstack
        psl = self._padstack_copper_layer(padstack, layer)
        if psl is None:
            if self._pad_is_through_hole(pad):
                # Mechanical hole without any copper: draw the drill.
                return self.parse_bare_hole(pad)
            self.logger.info("Pad %s has no copper layers, skipping.",
                             pad.number)
            return None
        pos = self.normalize(pad.position)
        size = self.normalize(psl.size)
        angle = padstack.angle.degrees
        shape = {
            PAD_STACK_SHAPE.PSS_CIRCLE: "circle",
            PAD_STACK_SHAPE.PSS_RECTANGLE: "rect",
            PAD_STACK_SHAPE.PSS_OVAL: "oval",
            PAD_STACK_SHAPE.PSS_TRAPEZOID: "trapezoid",
            PAD_STACK_SHAPE.PSS_ROUNDRECT: "roundrect",
            PAD_STACK_SHAPE.PSS_CHAMFEREDRECT: "chamfrect",
            PAD_STACK_SHAPE.PSS_CUSTOM: "custom",
        }.get(psl.shape, "")
        if shape == "":
            self.logger.info("Unsupported pad shape %s, skipping.", psl.shape)
            return None
        pad_dict = {
            "pos": pos,
            "size": size,
            "angle": angle,
            "shape": shape,
            "number": pad.number,
        }
        if shape == "custom":
            polygon = self.board.get_pad_shapes_as_polygons(pad, layer)
            if polygon is None:
                self.logger.info(
                    "Failed to get polygon for custom pad %s", pad.number)
                return None
            if polygon.holes:
                self.logger.warn('Detected holes in custom pad polygons')
            # Polygon comes in absolute board coordinates, convert it to
            # pad-relative unrotated coordinates that the renderer expects.
            contours = []
            for contour in self.parse_polygons([polygon]):
                contour = [self.rotate(p, pos, -angle) for p in contour]
                contours.append([[p[0] - pos[0], p[1] - pos[1]]
                                 for p in contour])
            pad_dict["polygons"] = contours
        if shape == "trapezoid":
            # treat trapezoid as custom shape
            pad_dict["shape"] = "custom"
            delta = self.normalize(psl.trapezoid_delta)
            pad_dict["polygons"] = [[
                [size[0] / 2 + delta[1] / 2, size[1] / 2 - delta[0] / 2],
                [-size[0] / 2 - delta[1] / 2, size[1] / 2 + delta[0] / 2],
                [-size[0] / 2 + delta[1] / 2, -size[1] / 2 - delta[0] / 2],
                [size[0] / 2 - delta[1] / 2, -size[1] / 2 + delta[0] / 2],
            ]]

        if shape in ["roundrect", "chamfrect"]:
            pad_dict["radius"] = (psl.corner_rounding_ratio *
                                  min(psl.size.x, psl.size.y) * 1e-6)
        if shape == "chamfrect":
            corners = psl.chamfered_corners
            chamfpos = 0
            if corners.top_left:
                chamfpos |= CHAMFER_TOP_LEFT
            if corners.top_right:
                chamfpos |= CHAMFER_TOP_RIGHT
            if corners.bottom_left:
                chamfpos |= CHAMFER_BOTTOM_LEFT
            if corners.bottom_right:
                chamfpos |= CHAMFER_BOTTOM_RIGHT
            pad_dict["chamfpos"] = chamfpos
            pad_dict["chamfratio"] = psl.chamfer_ratio

        if self._pad_is_through_hole(pad):
            drill = padstack.drill
            pad_dict["type"] = "th"
            pad_dict["drillshape"] = {
                DRILL_SHAPE.DS_CIRCLE: "circle",
                DRILL_SHAPE.DS_OBLONG: "oblong",
            }.get(drill.shape, "circle")
            pad_dict["drillsize"] = self.normalize(drill.diameter)
        else:
            pad_dict["type"] = "smd"

        pad_dict["offset"] = self.normalize(psl.offset)
        if self.config.include_nets:
            pad_dict["net"] = pad.net.name

        return pad_dict

    def parse_bare_hole(self, pad):
        drill = pad.padstack.drill
        oblong = drill.shape == DRILL_SHAPE.DS_OBLONG
        drillsize = self.normalize(drill.diameter)
        pad_dict = {
            "pos": self.normalize(pad.position),
            "size": drillsize,
            "angle": pad.padstack.angle.degrees,
            "shape": "oval" if oblong else "circle",
            "type": "th",
            "drillshape": "oblong" if oblong else "circle",
            "drillsize": drillsize,
            "offset": [0, 0],
            "number": pad.number,
        }
        if self.config.include_nets:
            pad_dict["net"] = pad.net.name
        return pad_dict

    def _footprint_bbox(self, f):
        """Footprint bounding box at orientation 0, relative to the
        footprint position, like the old parser computed by copying the
        footprint and resetting its orientation."""
        pos = self.normalize(f.position)
        angle = f.orientation.degrees
        points = []

        for d in f.definition.shapes:
            if d.layer == BL.BL_Edge_Cuts:
                continue
            points.extend(self._shape_points_with_extents(d))

        for pad in f.definition.pads:
            pad_pos = self.normalize(pad.position)
            half_w = half_h = 0
            for psl in pad.padstack.copper_layers:
                half_w = max(half_w, psl.size.x * 1e-6 / 2)
                half_h = max(half_h, psl.size.y * 1e-6 / 2)
            drill = pad.padstack.drill
            half_w = max(half_w, drill.diameter.x * 1e-6 / 2)
            half_h = max(half_h, drill.diameter.y * 1e-6 / 2)
            pad_angle = pad.padstack.angle.degrees
            for sx in (-1, 1):
                for sy in (-1, 1):
                    corner = [pad_pos[0] + sx * half_w,
                              pad_pos[1] + sy * half_h]
                    points.append(self.rotate(corner, pad_pos, pad_angle))

        if not points:
            points = [pos]

        # Board coordinates -> footprint-local unrotated coordinates.
        local = [self.rotate(p, pos, -angle) for p in points]
        minx = min(p[0] for p in local)
        miny = min(p[1] for p in local)
        maxx = max(p[0] for p in local)
        maxy = max(p[1] for p in local)

        return {
            "pos": pos,
            "relpos": [minx - pos[0], miny - pos[1]],
            "size": [maxx - minx, maxy - miny],
            "angle": angle,
        }

    def parse_footprints(self):
        footprints = []
        for f in self.footprints:
            ref = f.reference_field.text.value

            # graphical drawings on copper layers
            drawings = []
            copper_items = [d for d in f.definition.shapes
                            if d.layer in [BL.BL_F_Cu, BL.BL_B_Cu]]
            for d in copper_items:
                for drawing in self.parse_drawing(d, f):
                    drawings.append({
                        "layer": "F" if d.layer == BL.BL_F_Cu else "B",
                        "drawing": drawing,
                    })

            # footprint pads
            pads = []
            for p in f.definition.pads:
                for pad_dict in self.parse_pad(p):
                    pads.append((p.number, pad_dict))

            if pads:
                # Try to guess first pin name.
                pads = sorted(pads, key=lambda el: el[0])
                pin1_pads = [p for p in pads if p[0] in
                             ['1', 'A', 'A1', 'P1', 'PAD1']]
                if pin1_pads:
                    pin1_pad_name = pin1_pads[0][0]
                else:
                    # No pads have common first pin name,
                    # pick lexicographically smallest.
                    pin1_pad_name = pads[0][0]
                for pad_name, pad_dict in pads:
                    if pad_name == pin1_pad_name:
                        pad_dict['pin1'] = 1

            pads = [p[1] for p in pads]

            footprint_dict = {
                "ref": ref,
                "bbox": self._footprint_bbox(f),
                "pads": pads,
                "drawings": drawings,
                "layer": self.layer_letter(f.layer),
            }
            if self._footprint_is_dnp(f):
                footprint_dict["dnp"] = 1
            fields = self.footprint_info_fields(f)
            if fields:
                footprint_dict["fields"] = fields
            footprints.append(footprint_dict)

        return footprints

    # ------------------------------------------------------------------
    # tracks, vias, zones, nets

    @staticmethod
    def _via_untented(padstack, layer):
        outer = (padstack.front_outer_layers if layer == BL.BL_F_Cu
                 else padstack.back_outer_layers)
        # SMM_FROM_DESIGN_RULES follows the board setting, which tents
        # vias by default in KiCad, so only explicit unmasking counts.
        return outer.solder_mask_mode == SOLDER_MASK_MODE.SMM_UNMASKED

    def parse_track(self, track):
        """Track or arc track (not via) to a track dict."""
        if isinstance(track, ArcTrack):
            center = track.center()
            if center is None:
                return None
            a1, a2 = self.arc_angles_degrees(track)
            track_dict = {
                "center": self.normalize(center),
                "startangle": a1,
                "endangle": a2,
                "radius": math.hypot(track.start.x - center.x,
                                     track.start.y - center.y) * 1e-6,
                "width": track.width * 1e-6,
            }
        else:
            track_dict = {
                "start": self.normalize(track.start),
                "end": self.normalize(track.end),
                "width": track.width * 1e-6,
            }
        if self.config.include_nets:
            track_dict["net"] = track.net.name
        return track_dict

    def parse_tracks(self, tracks_and_vias):
        result = {BL.BL_F_Cu: [], BL.BL_B_Cu: []}
        for track in tracks_and_vias:
            if isinstance(track, Via):
                padstack = track.padstack
                pos = self.normalize(track.position)
                for layer, _ in OUTER_COPPER_LAYERS:
                    if layer not in padstack.layers:
                        continue
                    psl = padstack.copper_layer(layer)
                    if psl is None and padstack.copper_layers:
                        psl = padstack.copper_layers[0]
                    if psl is None:
                        continue
                    via_dict = {
                        "start": pos,
                        "end": pos,
                        "width": psl.size.x * 1e-6,
                        "net": track.net.name,
                        # Hole size, used to draw vias like KiCad does.
                        "drill": track.drill_diameter * 1e-6,
                    }
                    if self._via_untented(padstack, layer):
                        via_dict["drillsize"] = track.drill_diameter * 1e-6
                    result[layer].append(via_dict)
            else:
                if track.layer not in result:
                    continue
                track_dict = self.parse_track(track)
                if track_dict is not None:
                    result[track.layer].append(track_dict)

        return {
            'F': result[BL.BL_F_Cu],
            'B': result[BL.BL_B_Cu],
        }

    def parse_zones(self, zones):
        result = {BL.BL_F_Cu: [], BL.BL_B_Cu: []}
        for zone in zones:
            if not zone.filled or zone.is_rule_area():
                continue
            for layer, polys in zone.filled_polygons.items():
                if layer not in result:
                    continue
                zone_dict = {
                    "polygons": self.parse_polygons(polys),
                    # Since KiCad 7 fills are exact, no outline stroke.
                    "width": 0,
                }
                if self._has_holes(polys):
                    zone_dict["fillrule"] = "evenodd"
                if self.config.include_nets:
                    net = getattr(zone, 'net', None)
                    zone_dict["net"] = net.name if net is not None else ""
                result[layer].append(zone_dict)

        return {
            'F': result[BL.BL_F_Cu],
            'B': result[BL.BL_B_Cu],
        }

    def inner_copper_layers(self, tracks_and_vias, zones):
        """[(BoardLayer, name)] of the board's inner copper layers."""
        layers = None
        try:
            count = self.board.get_copper_layer_count()
            layers = INNER_COPPER_LAYERS[:max(0, count - 2)]
        except Exception:
            # Older KiCad / kicad-python: use the layers that have items.
            used = set(t.layer for t in tracks_and_vias
                       if not isinstance(t, Via))
            for z in zones:
                used.update(z.filled_polygons.keys())
            layers = [l for l in INNER_COPPER_LAYERS if l[0] in used]
        result = []
        for layer, name in layers:
            try:
                # User defined layer names, KiCad 9.0.8+.
                name = self.board.get_layer_name(layer) or name
            except Exception:
                pass
            result.append((layer, name))
        return result

    @staticmethod
    def _padstack_spans_layer(padstack, layer):
        """Whether a padstack (via/pad) has copper on an inner layer,
        ignoring unconnected layer removal."""
        layers = set(padstack.layers)
        if layer in layers:
            return True
        if layers & INNER_COPPER_LAYER_IDS:
            # Explicit layer list that does not include this layer.
            return False
        drill = padstack.drill
        span = [drill.start_layer, drill.end_layer]
        if all(l in INNER_COPPER_LAYER_IDS or
               l in (BL.BL_F_Cu, BL.BL_B_Cu) for l in span):
            return min(span) <= layer <= max(span)
        return BL.BL_F_Cu in layers and BL.BL_B_Cu in layers

    def _inner_presence(self, items, layers):
        """{item: {layer: bool}} for items whose copper on inner layers
        may be removed (unconnected layer removal / custom padstacks)."""
        items = [i for i in items
                 if i.padstack.unconnected_layer_removal not in
                 [ULR.ULR_KEEP, ULR.ULR_UNKNOWN] or
                 i.padstack.type == PAD_STACK_TYPE.PST_CUSTOM]
        if not items or not layers:
            return {}
        try:
            return self.board.check_padstack_presence_on_layers(
                items, layers)
        except Exception as e:
            self.logger.info('Padstack presence check failed: %s' % e)
            return {}

    def parse_inner_layers(self, tracks_and_vias, zones):
        """Copper of the inner layers, in stack order (In1 first)."""
        layers = self.inner_copper_layers(tracks_and_vias, zones)
        if not layers:
            return []
        layer_ids = [layer for layer, _ in layers]
        result = {layer: {"name": name, "tracks": [], "zones": [],
                          "pads": []}
                  for layer, name in layers}

        vias = [t for t in tracks_and_vias if isinstance(t, Via)]
        for track in tracks_and_vias:
            if not isinstance(track, Via) and track.layer in result:
                track_dict = self.parse_track(track)
                if track_dict is not None:
                    result[track.layer]["tracks"].append(track_dict)

        presence = self._inner_presence(vias, layer_ids)
        for via in vias:
            padstack = via.padstack
            pos = self.normalize(via.position)
            for layer in layer_ids:
                if not self._padstack_spans_layer(padstack, layer):
                    continue
                if not presence.get(via, {}).get(layer, True):
                    continue
                psl = self._padstack_copper_layer(padstack, layer)
                if psl is None:
                    continue
                result[layer]["tracks"].append({
                    "start": pos,
                    "end": pos,
                    "width": psl.size.x * 1e-6,
                    "net": via.net.name,
                    "drill": via.drill_diameter * 1e-6,
                    # No solder mask on inner layers.
                    "drillsize": via.drill_diameter * 1e-6,
                })

        for zone in zones:
            if not zone.filled or zone.is_rule_area():
                continue
            for layer, polys in zone.filled_polygons.items():
                if layer not in result:
                    continue
                zone_dict = {
                    "polygons": self.parse_polygons(polys),
                    "width": 0,
                }
                if self._has_holes(polys):
                    zone_dict["fillrule"] = "evenodd"
                if self.config.include_nets:
                    net = getattr(zone, 'net', None)
                    zone_dict["net"] = net.name if net is not None else ""
                result[layer]["zones"].append(zone_dict)

        pads = [p for f in self.footprints for p in f.definition.pads
                if p.pad_type == PAD_TYPE.PT_PTH]
        presence = self._inner_presence(pads, layer_ids)
        for pad in pads:
            for layer in layer_ids:
                if not self._padstack_spans_layer(pad.padstack, layer):
                    continue
                if not presence.get(pad, {}).get(layer, True):
                    continue
                pad_dict = self.parse_pad_layer(pad, layer)
                if pad_dict is not None:
                    pad_dict["layers"] = []
                    result[layer]["pads"].append(pad_dict)

        return [result[layer] for layer in layer_ids]

    def parse_netlist(self):
        return sorted(n.name for n in self.board.get_nets())

    # ------------------------------------------------------------------
    # fields, variants and BOM components

    def _footprint_variant(self, footprint):
        """Returns the kipy FootprintVariant for config.kicad_variant,
        or None for the default variant."""
        name = getattr(self.config, 'kicad_variant', '')
        if not name:
            return None
        if not hasattr(footprint, 'variants'):
            self._report_once(
                'no-variants',
                'Variant "%s" requested but this kicad-python / KiCad '
                'version does not expose footprint variants over the IPC '
                'API, using the default variant.' % name)
            return None
        for variant in footprint.variants:
            if variant.name == name:
                return variant
        return None

    def _footprint_is_dnp(self, footprint):
        variant = self._footprint_variant(footprint)
        if variant is not None:
            return bool(variant.do_not_populate)
        return bool(footprint.attributes.do_not_populate)

    def get_footprint_fields(self, f):
        props = {}
        for t in f.texts_and_fields:
            if isinstance(t, Field):
                props[t.name] = t.text.value
        variant = self._footprint_variant(f)
        if variant is not None:
            for k, v in variant.fields.items():
                props[str(k)] = str(v)
        keys = [k for k, v in props.items() if '${' in v]
        if keys:
            # Only fields with text variables need a round trip to KiCad.
            expanded = self.board.expand_text_variables(
                [props[k] for k in keys])
            props.update(zip(keys, expanded))
        if "dnp" in props and props["dnp"] == "":
            del props["dnp"]
            props["kicad_dnp"] = "DNP"
        if f.attributes.do_not_populate:
            props["kicad_dnp"] = "DNP"
        if variant is not None:
            props["kicad_dnp"] = "DNP" if variant.do_not_populate else ""
        return props

    def footprint_info_fields(self, f):
        """Non-empty fields of a footprint as [[name, value], ...] for the
        part info panel, in KiCad's order: Value, Footprint, Datasheet,
        Description, then the other fields."""
        props = self.get_footprint_fields(f)
        footprint_id = f.definition.id
        footprint_name = str(footprint_id.name)
        if footprint_id.library:
            footprint_name = "%s:%s" % (footprint_id.library, footprint_name)
        props["Footprint"] = footprint_name
        skip = {"reference", "dnp", "kicad_dnp"}
        first = ["value", "footprint", "datasheet", "description"]
        names = [k for k in props if k.lower() not in skip]
        names.sort(key=lambda k: first.index(k.lower())
                   if k.lower() in first else len(first))
        result = []
        for name in names:
            value = str(props[name]).strip()
            if value and value != "~":  # "~" is KiCad's empty value
                result.append([name, value])
        return result

    def parse_extra_data_from_pcb(self):
        field_set = set()
        by_ref = {}
        by_index = {}

        for (i, f) in enumerate(self.footprints):
            props = self.get_footprint_fields(f)
            by_index[i] = props
            ref = f.reference_field.text.value
            ref_fields = by_ref.setdefault(ref, {})

            for k, v in props.items():
                field_set.add(k)
                ref_fields[k] = v

        return ExtraFieldData(list(field_set), by_ref, by_index)

    def get_extra_field_data(self, file_name):
        if os.path.abspath(file_name) == os.path.abspath(self.file_name):
            return self.parse_extra_data_from_pcb()
        if os.path.splitext(file_name)[1] == '.kicad_pcb':
            return None

        data = parse_schematic_data(file_name)
        if data is None:
            return None

        return ExtraFieldData(data[0], data[1])

    def latest_extra_data(self, extra_dirs=None):
        base_name = os.path.splitext(os.path.basename(self.file_name))[0]
        file_dir_name = os.path.dirname(self.file_name)
        directories = [file_dir_name]
        for dir in (extra_dirs or []):
            if not os.path.isabs(dir):
                dir = os.path.join(file_dir_name, dir)
            if os.path.exists(dir):
                directories.append(dir)
        return find_latest_schematic_data(base_name, directories)

    def extra_data_file_filter(self):
        return ("Netlist, xml and pcb files (*.net; *.xml; *.kicad_pcb)|"
                "*.net;*.xml;*.kicad_pcb")

    def footprint_to_component(self, footprint, extra_fields):
        footprint_name = str(footprint.definition.id.name)

        variant = self._footprint_variant(footprint)
        if variant is not None:
            excluded = variant.exclude_from_bill_of_materials
        else:
            excluded = footprint.attributes.exclude_from_bill_of_materials
        attr = 'Virtual' if excluded else 'Normal'

        return Component(footprint.reference_field.text.value,
                         self._footprint_value(footprint),
                         footprint_name,
                         self.layer_letter(footprint.layer),
                         attr,
                         extra_fields)

    # ------------------------------------------------------------------

    def parse(self):
        # Get extra field data
        field_set = set(self.config.show_fields)
        field_set.discard("Value")
        field_set.discard("Footprint")
        need_extra_fields = (field_set or
                             self.config.board_variant_whitelist or
                             self.config.board_variant_blacklist or
                             self.config.dnp_field)

        if not self.config.extra_data_file and need_extra_fields:
            self.config.extra_data_file = self.file_name
            self.logger.warn('Assuming extra data file to be the pcb file '
                             'since --extra-data-file was not specified.')

        extra_field_data = None
        if self.config.extra_data_file:
            is_board = (os.path.abspath(self.config.extra_data_file) ==
                        os.path.abspath(self.file_name))
            if is_board or os.path.isfile(self.config.extra_data_file):
                extra_field_data = self.parse_extra_data(
                    self.config.extra_data_file,
                    self.config.normalize_field_case)

        if extra_field_data is None and need_extra_fields:
            raise ParsingException(
                'Failed parsing %s' % self.config.extra_data_file)

        title_block = self.board.get_title_block_info()
        title, revision, company, file_date = \
            self.board.expand_text_variables([
                title_block.title,
                title_block.revision,
                title_block.company,
                title_block.date,
            ])

        if not file_date and os.path.isfile(self.file_name):
            file_mtime = os.path.getmtime(self.file_name)
            file_date = datetime.fromtimestamp(file_mtime).strftime(
                '%Y-%m-%d %H:%M:%S')
        pcb_file_name = os.path.basename(self.file_name)
        if not title:
            # remove .kicad_pcb extension
            title = os.path.splitext(pcb_file_name)[0]

        edges, bbox = self.parse_edges()
        if bbox is None:
            self.logger.error('Please draw pcb outline on the edges '
                              'layer on sheet or any footprint before '
                              'generating BOM.')
            return None, None
        bbox = {
            "minx": bbox[0],
            "miny": bbox[1],
            "maxx": bbox[2],
            "maxy": bbox[3],
        }

        self.report_unsupported_items()
        drawings = self.get_all_drawings()

        pcbdata = {
            "edges_bbox": bbox,
            "edges": edges,
            "drawings": {
                "silkscreen": self.parse_drawings_on_layers(
                    drawings, BL.BL_F_SilkS, BL.BL_B_SilkS),
                "fabrication": self.parse_drawings_on_layers(
                    drawings, BL.BL_F_Fab, BL.BL_B_Fab),
            },
            "footprints": self.parse_footprints(),
            "metadata": {
                "title": title,
                "revision": revision,
                "company": company,
                "date": file_date,
                "variant": getattr(self.config, 'kicad_variant', ''),
            },
            "bom": {},
            # All text is rendered to strokes/polygons by KiCad.
            "font_data": {}
        }
        # Note: "board_clip" (castellated holes) is optional and needs
        # board outline polygon operations that the IPC API does not
        # offer yet, so it is omitted and the classic rendering is used.
        if self.config.include_tracks:
            tracks_and_vias = (list(self.board.get_tracks()) +
                               list(self.board.get_vias()))
            zones = list(self.board.get_zones())
            pcbdata["tracks"] = self.parse_tracks(tracks_and_vias)
            pcbdata["zones"] = self.parse_zones(zones)
            pcbdata["inner_layers"] = self.parse_inner_layers(
                tracks_and_vias, zones)
        if self.config.include_nets:
            pcbdata["nets"] = self.parse_netlist()

        if extra_field_data and need_extra_fields:
            extra_fields = extra_field_data.fields_by_index
            if extra_fields:
                extra_fields = extra_fields.values()

            if extra_fields is None:
                extra_fields = []
                field_map = extra_field_data.fields_by_ref
                warning_shown = False

                for f in self.footprints:
                    ref = f.reference_field.text.value
                    extra_fields.append(field_map.get(ref, {}))
                    if ref not in field_map:
                        # Some components are on pcb but not in schematic
                        # data. Show a warning about outdated extra data
                        # file.
                        self.logger.warn(
                            'Component %s is missing from schematic data.'
                            % ref)
                        warning_shown = True

                if warning_shown:
                    self.logger.warn('Netlist/xml file is likely out of date.')
        else:
            extra_fields = [{}] * len(self.footprints)

        components = [self.footprint_to_component(f, e)
                      for (f, e) in zip(self.footprints, extra_fields)]

        return pcbdata, components
