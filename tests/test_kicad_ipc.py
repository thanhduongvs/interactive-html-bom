"""Tests for the KiCad IPC API parser.

These tests do not need a running KiCad: the board is assembled from real
kicad-python (kipy) item objects built from protobuf messages, and served
by a small fake Board / KiCad pair that implements the handful of API calls
the parser uses.
"""
import io
import json
import os

import pytest

pytest.importorskip("kipy")

from google.protobuf.any_pb2 import Any  # noqa: E402
from kipy.common_types import CompoundShape, TitleBlockInfo  # noqa: E402
from kipy.geometry import PolygonWithHoles  # noqa: E402
from kipy.proto.board import board_types_pb2 as bpb  # noqa: E402
from kipy.proto.common.types import base_types_pb2 as cpb  # noqa: E402
from kipy.board_types import (  # noqa: E402
    BoardShape, to_concrete_board_shape, unwrap)

os.environ.setdefault('INTERACTIVE_HTML_BOM_NO_DISPLAY', '1')

from InteractiveHtmlBom.core import ibom  # noqa: E402
from InteractiveHtmlBom.core.config import Config  # noqa: E402
from InteractiveHtmlBom.ecad.kicad_ipc import KiCadIPCParser  # noqa: E402

BL = bpb.BoardLayer
MM = 1000000


def v(x, y):
    return cpb.Vector2(x_nm=int(round(x * MM)), y_nm=int(round(y * MM)))


def dist(mm):
    return cpb.Distance(value_nm=int(round(mm * MM)))


def wrap(msg):
    """Wraps a proto into a kipy object the same way Board.get_items does."""
    a = Any()
    a.Pack(msg)
    item = unwrap(a)
    if isinstance(item, BoardShape):
        item = to_concrete_board_shape(item)
    return item


def packed(msg):
    a = Any()
    a.Pack(msg)
    return a


def stroke(width=0.1, filled=False):
    return cpb.GraphicAttributes(
        stroke=cpb.StrokeAttributes(width=dist(width)),
        fill=cpb.GraphicFillAttributes(
            fill_type=cpb.GraphicFillType.GFT_FILLED if filled
            else cpb.GraphicFillType.GFT_UNFILLED))


def segment(layer, a, b, width=0.1):
    return bpb.BoardGraphicShape(layer=layer, shape=cpb.GraphicShape(
        attributes=stroke(width),
        segment=cpb.GraphicSegmentAttributes(start=v(*a), end=v(*b))))


def arc(layer, a, m, b, width=0.1):
    return bpb.BoardGraphicShape(layer=layer, shape=cpb.GraphicShape(
        attributes=stroke(width),
        arc=cpb.GraphicArcAttributes(start=v(*a), mid=v(*m), end=v(*b))))


def poly_line(points, arcs=()):
    nodes = [cpb.PolyLineNode(point=v(*p)) for p in points]
    for a in arcs:
        nodes.append(cpb.PolyLineNode(arc=cpb.ArcStartMidEnd(
            start=v(*a[0]), mid=v(*a[1]), end=v(*a[2]))))
    return cpb.PolyLine(nodes=nodes, closed=True)


def text(value, pos, layer, size=1.0):
    return bpb.BoardText(layer=layer, text=cpb.Text(
        text=value, position=v(*pos),
        attributes=cpb.TextAttributes(
            size=v(size, size), stroke_width=dist(0.15), visible=True)))


def field(field_id, name, value, pos, layer=BL.BL_F_SilkS, visible=True):
    return bpb.Field(id=bpb.FieldId(id=field_id), name=name,
                     visible=visible, text=text(value, pos, layer))


def padstack(layers, shape, size, angle=0, drill=None, stack_type=None,
             copper=None, **layer_props):
    stack = bpb.PadStack(
        type=stack_type or bpb.PadStackType.PST_NORMAL,
        layers=layers,
        angle=cpb.Angle(value_degrees=angle),
        unconnected_layer_removal=bpb.UnconnectedLayerRemoval.ULR_KEEP,
        copper_layers=copper if copper is not None else [bpb.PadStackLayer(
            layer=BL.BL_F_Cu, shape=shape, size=v(*size), **layer_props)])
    if drill:
        stack.drill.CopyFrom(bpb.DrillProperties(
            diameter=v(*drill), shape=bpb.DrillShape.DS_CIRCLE))
    return stack


def pad(number, pos, stack, pad_type=bpb.PadType.PT_SMD, net="GND"):
    return bpb.Pad(number=number, position=v(*pos), type=pad_type,
                   pad_stack=stack, net=bpb.Net(name=net))


def footprint(ref, value, pos, angle, items, layer=BL.BL_F_Cu,
              dnp=False, exclude_from_bom=False):
    x, y = pos
    return bpb.FootprintInstance(
        position=v(x, y),
        orientation=cpb.Angle(value_degrees=angle),
        layer=layer,
        reference_field=field(1, "Reference", ref, (x, y - 1.5)),
        value_field=field(2, "Value", value, (x, y + 1.5), BL.BL_F_Fab),
        datasheet_field=field(3, "Datasheet", "~" if ref == "J1" else "",
                              (x, y), visible=False),
        description_field=field(4, "Description", "", (x, y),
                                visible=False),
        attributes=bpb.FootprintAttributes(
            do_not_populate=dnp,
            exclude_from_bill_of_materials=exclude_from_bom),
        definition=bpb.Footprint(
            id=cpb.LibraryIdentifier(library_nickname="Lib",
                                     entry_name="FP_" + ref),
            items=[packed(i) for i in items]))


class FakeProject(object):
    def __init__(self, path):
        self.path = path


class FakeBoard(object):
    def __init__(self, path, items):
        self.name = "test.kicad_pcb"
        self._project = FakeProject(path)
        self.items = items
        self.presence_calls = []

    def get_project(self):
        return self._project

    def get_copper_layer_count(self):
        return 4

    def get_footprints(self):
        return self.items['footprints']

    def get_shapes(self):
        return self.items['shapes']

    def get_text(self):
        return self.items['text']

    def get_tracks(self):
        return self.items['tracks']

    def get_vias(self):
        return self.items['vias']

    def get_zones(self):
        return self.items['zones']

    def get_nets(self):
        from kipy.board_types import Net
        return [Net(bpb.Net(name=n)) for n in ["VCC", "", "GND"]]

    def get_title_block_info(self):
        return TitleBlockInfo(cpb.TitleBlockInfo(
            title="${PROJECTNAME}", revision="A", company="ACME",
            date="2026-11-12"))

    def expand_text_variables(self, t):
        def expand(s):
            return s.replace("${PROJECTNAME}", "Test board")
        if isinstance(t, list):
            return [expand(s) for s in t]
        return expand(t)

    def get_pad_shapes_as_polygons(self, pad, layer=BL.BL_F_Cu):
        # A 1x1 mm square around the pad, in absolute coordinates.
        x = pad.position.x * 1e-6
        y = pad.position.y * 1e-6
        return PolygonWithHoles(cpb.PolygonWithHoles(outline=poly_line(
            [(x - .5, y - .5), (x + .5, y - .5), (x + .5, y + .5),
             (x - .5, y + .5)])))

    def check_padstack_presence_on_layers(self, items, layers):
        self.presence_calls.append(items)
        return {items: {BL.BL_F_Cu: True, BL.BL_B_Cu: False}}


class FakeKiCad(object):
    def __init__(self):
        self.calls = 0
        self.rendered = []

    def get_text_as_shapes(self, texts):
        self.calls += 1
        if not isinstance(texts, list):
            texts = [texts]
        result = []
        for t in texts:
            self.rendered.append(t.value)
            p = t.position
            result.append(CompoundShape(cpb.CompoundShape(shapes=[
                cpb.GraphicShape(segment=cpb.GraphicSegmentAttributes(
                    start=p.proto, end=v(p.x * 1e-6 + 1, p.y * 1e-6)))])))
        return result


class FakeVariant(object):
    def __init__(self, name, dnp=False, exclude=False, fields=None):
        self.name = name
        self.do_not_populate = dnp
        self.exclude_from_bill_of_materials = exclude
        self.fields = fields or {}


def make_board(tmp_path):
    edge = BL.BL_Edge_Cuts
    shapes = [wrap(s) for s in [
        segment(edge, (0, 0), (50, 0)),
        segment(edge, (50, 0), (50, 30)),
        segment(edge, (50, 30), (0, 30)),
        segment(edge, (0, 30), (0, 0)),
        arc(BL.BL_F_SilkS, (10, 10), (12, 8), (14, 10), 0.2),
        bpb.BoardGraphicShape(layer=BL.BL_F_SilkS, shape=cpb.GraphicShape(
            attributes=stroke(0.1, filled=True),
            polygon=cpb.PolySet(polygons=[cpb.PolygonWithHoles(
                outline=poly_line([(30, 20), (35, 20)],
                                  arcs=[((35, 20), (36, 21), (35, 22))]),
                holes=[poly_line([(31, 20.5), (32, 20.5), (32, 21)])])]))),
    ]]

    smd = padstack([BL.BL_F_Cu, BL.BL_F_Mask], bpb.PadStackShape.PSS_ROUNDRECT,
                   (1, 0.8), angle=90, corner_rounding_ratio=0.25)
    th = padstack([BL.BL_F_Cu, BL.BL_B_Cu], bpb.PadStackShape.PSS_CIRCLE,
                  (1.6, 1.6), drill=(0.8, 0.8))
    custom = padstack([BL.BL_F_Cu], bpb.PadStackShape.PSS_CUSTOM, (1, 1))
    fib = padstack([BL.BL_F_Cu, BL.BL_B_Cu], bpb.PadStackShape.PSS_CIRCLE,
                   (1.2, 1.2), drill=(0.6, 0.6),
                   stack_type=bpb.PadStackType.PST_FRONT_INNER_BACK)
    npth_no_copper = padstack([], bpb.PadStackShape.PSS_CIRCLE, (2, 2),
                              drill=(2, 2), copper=[])

    footprints = [wrap(f) for f in [
        footprint("R1", "10k", (10, 15), 90, [
            pad("1", (10, 14.2), smd),
            pad("2", (10, 15.8), smd, net="VCC"),
            segment(BL.BL_F_Fab, (9.5, 14), (10.5, 14)),
            field(10, "MPN", "RC0603", (10, 15), visible=False),
            field(11, "LCSC", "C25804", (10, 15), visible=False),
            text("${REFERENCE}", (10, 17), BL.BL_F_SilkS),
        ]),
        footprint("J1", "Conn", (25, 15), 0, [
            field(20, "Category", "Connector", (25, 15), visible=False),
            field(21, "SubCategory", "", (25, 15), visible=False),
            field(22, "Manufacturer Part", "B2B-XH-A", (25, 15),
                  visible=False),
            field(23, "Distributor", "LCSC", (25, 15), visible=False),
            field(24, "Distributor Part", "C158012", (25, 15),
                  visible=False),
            pad("1", (25, 15), th, bpb.PadType.PT_PTH),
            pad("2", (27.54, 15), th, bpb.PadType.PT_PTH, net="VCC"),
            pad("MH", (30, 15), npth_no_copper, bpb.PadType.PT_NPTH, net=""),
        ]),
        footprint("U1", "MCU", (40, 10), 0, [
            pad("A1", (40, 10), custom),
            pad("A2", (41, 10), fib, bpb.PadType.PT_PTH),
        ], dnp=True),
        footprint("H1", "Logo", (5, 25), 0, [], exclude_from_bom=True),
    ]]

    via_stack = bpb.PadStack(
        type=bpb.PadStackType.PST_NORMAL, layers=[BL.BL_F_Cu, BL.BL_B_Cu],
        copper_layers=[bpb.PadStackLayer(
            layer=BL.BL_F_Cu, shape=bpb.PadStackShape.PSS_CIRCLE,
            size=v(0.6, 0.6))],
        drill=bpb.DrillProperties(diameter=v(0.3, 0.3)),
        front_outer_layers=bpb.PadStackOuterLayer(
            solder_mask_mode=bpb.SolderMaskMode.SMM_UNMASKED),
        back_outer_layers=bpb.PadStackOuterLayer(
            solder_mask_mode=bpb.SolderMaskMode.SMM_FROM_DESIGN_RULES))

    zone = bpb.Zone(
        type=bpb.ZoneType.ZT_COPPER, layers=[BL.BL_B_Cu], filled=True,
        copper_settings=bpb.CopperZoneSettings(net=bpb.Net(name="GND")),
        filled_polygons=[bpb.ZoneFilledPolygons(
            layer=BL.BL_B_Cu, shapes=cpb.PolySet(polygons=[
                cpb.PolygonWithHoles(
                    outline=poly_line([(1, 1), (49, 1), (49, 29), (1, 29)]),
                    holes=[poly_line([(20, 10), (22, 10), (22, 12)])])]))])
    inner_zone = bpb.Zone(
        type=bpb.ZoneType.ZT_COPPER, layers=[BL.BL_In2_Cu], filled=True,
        copper_settings=bpb.CopperZoneSettings(net=bpb.Net(name="VCC")),
        filled_polygons=[bpb.ZoneFilledPolygons(
            layer=BL.BL_In2_Cu, shapes=cpb.PolySet(polygons=[
                cpb.PolygonWithHoles(outline=poly_line(
                    [(2, 2), (48, 2), (48, 28), (2, 28)]))]))])
    blind_stack = bpb.PadStack(
        type=bpb.PadStackType.PST_NORMAL, layers=[BL.BL_F_Cu],
        copper_layers=[bpb.PadStackLayer(
            layer=BL.BL_F_Cu, shape=bpb.PadStackShape.PSS_CIRCLE,
            size=v(0.4, 0.4))],
        drill=bpb.DrillProperties(diameter=v(0.2, 0.2),
                                  start_layer=BL.BL_F_Cu,
                                  end_layer=BL.BL_In2_Cu))
    micro_stack = bpb.PadStack()
    micro_stack.CopyFrom(blind_stack)
    micro_stack.drill.end_layer = BL.BL_In1_Cu
    rule_area = bpb.Zone(type=bpb.ZoneType.ZT_RULE_AREA,
                         layers=[BL.BL_F_Cu], filled=True)

    items = {
        'shapes': shapes,
        'text': [wrap(text("${PROJECTNAME}", (5, 5), BL.BL_F_SilkS)),
                 wrap(text("back", (5, 5), BL.BL_B_SilkS))],
        'footprints': footprints,
        'tracks': [
            wrap(bpb.Track(start=v(10, 14.2), end=v(25, 15), width=dist(0.25),
                           layer=BL.BL_F_Cu, net=bpb.Net(name="GND"))),
            wrap(bpb.Arc(start=v(10, 20), mid=v(12, 22), end=v(14, 20),
                         width=dist(0.25), layer=BL.BL_B_Cu,
                         net=bpb.Net(name="VCC"))),
            wrap(bpb.Track(start=v(1, 1), end=v(2, 2), width=dist(0.25),
                           layer=BL.BL_In1_Cu, net=bpb.Net(name="GND"))),
        ],
        'vias': [wrap(bpb.Via(position=v(20, 20), pad_stack=via_stack,
                              net=bpb.Net(name="GND"))),
                 wrap(bpb.Via(position=v(30, 25), pad_stack=blind_stack,
                              net=bpb.Net(name="VCC"))),
                 wrap(bpb.Via(position=v(35, 25), pad_stack=micro_stack,
                              net=bpb.Net(name="VCC")))],
        'zones': [wrap(zone), wrap(rule_area), wrap(inner_zone)],
    }
    return FakeBoard(str(tmp_path), items)


def make_parser(tmp_path, **config_values):
    config = Config("v2.12.0-test", str(tmp_path))
    config.include_tracks = True
    config.include_nets = True
    for k, val in config_values.items():
        setattr(config, k, val)
    board = make_board(tmp_path)
    kicad = FakeKiCad()
    logger = ibom.Logger(cli=True)
    parser = KiCadIPCParser(os.path.join(str(tmp_path), board.name),
                            config, logger, kicad, board)
    return parser, board, kicad


def load_schema():
    schema_file = os.path.join(os.path.dirname(__file__),
                               'pcbdata.schema.json')
    with io.open(schema_file, encoding='utf-8') as f:
        schema = json.load(f)
    # The schema comes from the former generic json input format; parser
    # output differs slightly: zones carry a stroke width and footprints
    # have no "center".
    schema['definitions']['Zone']['properties']['width'] = {
        "type": "number"}
    schema['definitions']['Footprint']['required'].remove('center')
    return schema


def components_json(components):
    return [{
        "ref": c.ref, "val": c.val, "footprint": c.footprint,
        "layer": c.layer, "attr": c.attr, "extra_fields": c.extra_fields,
    } for c in components]


def test_parse_matches_schema(tmp_path):
    jsonschema = pytest.importorskip("jsonschema")
    parser, _, _ = make_parser(tmp_path)
    pcbdata, components = parser.parse()
    pcbdata = dict(pcbdata)
    del pcbdata["bom"]  # filled in later by ibom.main()
    jsonschema.validate(
        instance={"spec_version": 1, "pcbdata": pcbdata,
                  "components": components_json(components)},
        schema=load_schema())


def test_board_geometry(tmp_path):
    parser, board, kicad = make_parser(tmp_path)
    pcbdata, _ = parser.parse()

    assert pcbdata["edges_bbox"] == pytest.approx(
        {"minx": -0.05, "miny": -0.05, "maxx": 50.05, "maxy": 30.05})
    assert len(pcbdata["edges"]) == 4
    assert pcbdata["metadata"]["title"] == "Test board"
    assert pcbdata["metadata"]["company"] == "ACME"
    assert pcbdata["nets"] == ["", "GND", "VCC"]

    silk_f = pcbdata["drawings"]["silkscreen"]["F"]
    arcs = [d for d in silk_f if d.get("type") == "arc"]
    assert len(arcs) == 1
    assert arcs[0]["start"] == pytest.approx([12, 10])
    assert arcs[0]["radius"] == pytest.approx(2)
    # Arc through the top of the circle on screen: from 180 to 360 deg.
    assert (arcs[0]["startangle"], arcs[0]["endangle"]) == (-180, 0)

    polys = [d for d in silk_f if d.get("type") == "polygon"]
    assert len(polys) == 1
    outline, hole = polys[0]["polygons"]
    assert len(hole) == 3
    # The arc node got approximated with several points.
    assert len(outline) > 6
    assert max(p[0] for p in outline) == pytest.approx(36, abs=1e-3)

    # Text is rendered by KiCad in two batched calls (silk + fab).
    assert kicad.calls == 2
    assert "Test board" in kicad.rendered
    assert "R1" in kicad.rendered  # ${REFERENCE} expanded locally
    assert "RC0603" not in kicad.rendered  # hidden field
    refs = [d for d in silk_f if "ref" in d]
    assert len(refs) == 4
    assert pcbdata["drawings"]["silkscreen"]["B"][0]["svgpath"]
    vals = [d for d in pcbdata["drawings"]["fabrication"]["F"] if "val" in d]
    assert len(vals) == 4


def test_footprints_and_pads(tmp_path):
    parser, board, _ = make_parser(tmp_path)
    pcbdata, _ = parser.parse()
    fps = {f["ref"]: f for f in pcbdata["footprints"]}

    r1 = fps["R1"]
    assert r1["layer"] == "F"
    assert r1["bbox"]["angle"] == 90
    assert r1["bbox"]["pos"] == pytest.approx([10, 15])
    p1, p2 = r1["pads"]
    assert p1["pin1"] == 1 and "pin1" not in p2
    assert (p1["number"], p2["number"]) == ("1", "2")
    assert "dnp" not in r1 and fps["U1"]["dnp"] == 1
    # Non-empty fields only (Datasheet/Description are empty), Value and
    # Footprint first, Reference left out.
    assert r1["fields"] == [["Value", "10k"], ["Footprint", "Lib:FP_R1"],
                            ["MPN", "RC0603"], ["LCSC", "C25804"]]
    # "~" datasheet and empty SubCategory are left out.
    assert fps["J1"]["fields"] == [
        ["Value", "Conn"], ["Footprint", "Lib:FP_J1"],
        ["Category", "Connector"], ["Manufacturer Part", "B2B-XH-A"],
        ["Distributor", "LCSC"], ["Distributor Part", "C158012"]]
    assert p1["shape"] == "roundrect" and p1["type"] == "smd"
    assert p1["radius"] == pytest.approx(0.2)
    assert p1["layers"] == ["F"]
    assert p1["net"] == "GND" and p2["net"] == "VCC"

    j1 = fps["J1"]
    assert len(j1["pads"]) == 3
    th = j1["pads"][0]
    assert th["type"] == "th" and th["layers"] == ["F", "B"]
    assert th["drillsize"] == pytest.approx([0.8, 0.8])
    # NPTH pad without copper is kept, like the pcbnew parser did.
    mh = [p for p in j1["pads"] if p["pos"] == pytest.approx([30, 15])][0]
    assert mh["layers"] == [] and mh["type"] == "th"
    assert mh["drillsize"] == pytest.approx([2, 2])
    assert mh["number"] == "MH"

    u1 = fps["U1"]
    custom = u1["pads"][0]
    assert custom["shape"] == "custom"
    flat = [c for p in custom["polygons"][0] for c in p]
    assert flat == pytest.approx([-.5, -.5, .5, -.5, .5, .5, -.5, .5])
    # Front/inner/back padstack: only present on front per KiCad.
    fib = [p for p in u1["pads"] if p["pos"] == pytest.approx([41, 10])]
    assert len(fib) == 1 and fib[0]["layers"] == ["F"]
    assert len(board.presence_calls) == 1


def test_tracks_vias_zones(tmp_path):
    parser, _, _ = make_parser(tmp_path)
    pcbdata, _ = parser.parse()
    tracks = pcbdata["tracks"]

    front_via = [t for t in tracks["F"] if t["start"] == t["end"] and
                 t["start"] == pytest.approx([20, 20])][0]
    back_via = [t for t in tracks["B"] if "start" in t and
                t["start"] == t["end"]][0]
    assert front_via["width"] == pytest.approx(0.6)
    assert front_via["drillsize"] == pytest.approx(0.3)  # unmasked
    assert "drillsize" not in back_via  # tented by design rules
    # Hole size is always exported for drawing vias.
    assert front_via["drill"] == pytest.approx(0.3)
    assert back_via["drill"] == pytest.approx(0.3)
    assert front_via["net"] == "GND"

    track = [t for t in tracks["F"] if t["start"] != t["end"]][0]
    assert track["width"] == pytest.approx(0.25)
    arc_track = [t for t in tracks["B"] if "center" in t][0]
    assert arc_track["center"] == pytest.approx([12, 20])
    assert arc_track["radius"] == pytest.approx(2)
    # In1 track is not on outer layers; blind vias only on front.
    assert len(tracks["F"]) == 4 and len(tracks["B"]) == 2

    zones = pcbdata["zones"]
    assert zones["F"] == []  # rule area skipped
    zone = zones["B"][0]
    assert zone["net"] == "GND" and zone["width"] == 0
    assert zone["fillrule"] == "evenodd"
    assert len(zone["polygons"]) == 2


def test_bom_components(tmp_path):
    parser, _, _ = make_parser(tmp_path, show_fields=["Value", "MPN", "LCSC"])
    _, components = parser.parse()
    comps = {c.ref: c for c in components}
    assert comps["R1"].val == "10k"
    assert comps["R1"].footprint == "FP_R1"
    assert comps["R1"].extra_fields["MPN"] == "RC0603"
    assert comps["R1"].extra_fields["LCSC"] == "C25804"
    assert comps["U1"].extra_fields["kicad_dnp"] == "DNP"
    assert comps["H1"].attr == "Virtual"
    assert comps["R1"].attr == "Normal"


def test_variants(tmp_path):
    parser, _, _ = make_parser(tmp_path, kicad_variant="LowCost",
                               show_fields=["Value", "MPN"])
    fps = {f.reference_field.text.value: f for f in parser.footprints}
    fps["R1"].variants = [FakeVariant("Other", dnp=True),
                          FakeVariant("LowCost",
                                      fields={"Value": "4k7",
                                              "MPN": "RC0402"})]
    fps["U1"].variants = [FakeVariant("LowCost", dnp=False)]
    fps["J1"].variants = [FakeVariant("LowCost", dnp=True, exclude=True)]
    fps["H1"].variants = []

    pcbdata, components = parser.parse()
    comps = {c.ref: c for c in components}
    assert pcbdata["metadata"]["variant"] == "LowCost"
    assert comps["R1"].val == "4k7"
    assert comps["R1"].extra_fields["MPN"] == "RC0402"
    assert comps["U1"].extra_fields["kicad_dnp"] == ""
    assert comps["J1"].extra_fields["kicad_dnp"] == "DNP"
    assert comps["J1"].attr == "Virtual"
    assert comps["H1"].attr == "Virtual"  # no override: base attributes
    fp_dnp = {f["ref"]: f.get("dnp", 0) for f in pcbdata["footprints"]}
    assert fp_dnp == {"R1": 0, "U1": 0, "J1": 1, "H1": 0}


def test_variant_requested_without_api_support(tmp_path):
    parser, _, _ = make_parser(tmp_path, kicad_variant="LowCost")
    _, components = parser.parse()
    assert {c.ref: c.val for c in components}["R1"] == "10k"


def test_generates_html(tmp_path):
    parser, _, _ = make_parser(tmp_path)
    parser.config.bom_dest_dir = str(tmp_path / "bom")
    parser.config.open_browser = False
    ibom.main(parser, parser.config, parser.logger)
    files = os.listdir(str(tmp_path / "bom"))
    assert files == ["html_test.html"]
    with io.open(str(tmp_path / "bom" / files[0]), encoding="utf-8") as f:
        html = f.read()
    assert "var pcbdata" in html


def test_extra_fields_from_netlist(tmp_path):
    netlist = tmp_path / "test.net"
    netlist.write_text(u"""(export (version "E")
  (components
    (comp (ref "R1") (value "10k")
      (fields (field (name "MPN") "FROM-NETLIST")))))
""")
    parser, _, _ = make_parser(tmp_path, show_fields=["Value", "MPN"],
                               extra_data_file=str(netlist))
    _, components = parser.parse()
    comps = {c.ref: c for c in components}
    assert comps["R1"].extra_fields.get("MPN") == "FROM-NETLIST"
    assert parser.latest_extra_data([]) is not None


def test_dialog_checks_kicad_dnp_by_default(tmp_path):
    config = Config("test", str(tmp_path))
    fields = ["Value", "Footprint", "MPN", "kicad_dnp"]
    assert config.checked_fields_for(fields) == (
        ["Value", "Footprint", "kicad_dnp"],
        ["Value", "Footprint", "kicad_dnp"])
    # Not added when the board has no DNP parts.
    assert config.checked_fields_for(fields[:3]) == (
        ["Value", "Footprint"], ["Value", "Footprint"])
    # A saved custom selection is respected.
    config.show_fields = ["Value", "MPN"]
    config.group_fields = ["Value"]
    assert config.checked_fields_for(fields) == (["Value", "MPN"], ["Value"])


def test_inner_layers(tmp_path):
    parser, _, _ = make_parser(tmp_path)
    pcbdata, _ = parser.parse()
    inner = pcbdata["inner_layers"]
    assert [l["name"] for l in inner] == ["In1.Cu", "In2.Cu"]
    in1, in2 = inner

    # Track on In1; through via and blind via F..In2 on both layers
    # (the blind via ends on In2, so In2 has its copper too).
    in1_tracks = [t for t in in1["tracks"] if t["start"] != t["end"]]
    assert len(in1_tracks) == 1 and in1_tracks[0]["net"] == "GND"
    vias1 = sorted(t["start"][0] for t in in1["tracks"]
                   if t["start"] == t["end"])
    assert vias1 == pytest.approx([20, 30, 35])
    through = [t for t in in1["tracks"]
               if t["start"] == pytest.approx([20, 20])][0]
    assert through["drillsize"] == pytest.approx(0.3)
    vias2 = [t for t in in2["tracks"] if t["start"] == t["end"]]
    assert sorted(t["start"][0] for t in vias2) == pytest.approx([20, 30])

    assert in1["zones"] == []
    assert in2["zones"][0]["net"] == "VCC"

    # Plated through hole pads of J1 (and U1's front/inner/back pad).
    in1_pads = sorted(p["pos"][0] for p in in1["pads"])
    assert in1_pads == pytest.approx([25, 27.54, 41])
    assert all(p["type"] == "th" for p in in1["pads"])


def test_inner_layers_only_with_tracks(tmp_path):
    parser, _, _ = make_parser(tmp_path, include_tracks=False)
    pcbdata, _ = parser.parse()
    assert "inner_layers" not in pcbdata
