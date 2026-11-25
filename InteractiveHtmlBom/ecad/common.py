class ExtraFieldData(object):
    def __init__(self, fields, fields_by_ref, fields_by_index=None):
        self.fields = fields
        self.fields_by_ref = fields_by_ref
        self.fields_by_index = fields_by_index


class EcadParser(object):

    def __init__(self, file_name, config, logger):
        """
        :param file_name: path to file that should be parsed.
        :param config: Config instance
        :param logger: logging object.
        """
        self.file_name = file_name
        self.config = config
        self.logger = logger

    def parse(self):
        """
        Abstract method that should be overridden in implementations.
        Performs all the parsing and returns a tuple of
        (pcbdata, components)
        pcbdata is described in DATAFORMAT.md
        components is list of Component objects
        :return:
        """
        pass

    @staticmethod
    def normalize_field_names(data):
        # type: (ExtraFieldData) -> ExtraFieldData
        def remap(ref_fields):
            return {f.lower(): v for (f, v) in
                    sorted(ref_fields.items(), reverse=True) if v}

        by_ref = {r: remap(d) for (r, d) in data.fields_by_ref.items()}
        if data.fields_by_index:
            by_index = {i: remap(d) for (i, d) in data.fields_by_index.items()}
        else:
            by_index = None

        field_map = {f.lower(): f for f in sorted(data.fields, reverse=True)}
        return ExtraFieldData(field_map.values(), by_ref, by_index)

    def get_extra_field_data(self, file_name):
        """
        Abstract method that may be overridden in implementations that support
        extra field data.
        :return: ExtraFieldData
        """
        return ExtraFieldData([], {})

    def parse_extra_data(self, file_name, normalize_case):
        """
        Parses the file and returns extra field data.
        :param file_name: path to file containing extra data
        :param normalize_case: if true, normalize case so that
                               "mpn", "Mpn", "MPN" fields are combined
        :return:
        """
        data = self.get_extra_field_data(file_name)
        if normalize_case:
            data = self.normalize_field_names(data)
        return ExtraFieldData(
            sorted(data.fields), data.fields_by_ref, data.fields_by_index)

    def latest_extra_data(self, extra_dirs=None):
        """
        Abstract method that may be overridden in implementations that support
        extra field data.
        :param extra_dirs: List of extra directories to search.
        :return: File name of most recent file with extra field data.
        """
        return None

    def extra_data_file_filter(self):
        """
        Abstract method that may be overridden in implementations that support
        extra field data.
        :return: File open dialog filter string, eg:
                 "Netlist and xml files (*.net; *.xml)|*.net;*.xml"
        """
        return None


class Component(object):
    """Simple data object to store component data needed for bom table."""

    def __init__(self, ref, val, footprint, layer, attr=None, extra_fields={}):
        self.ref = ref
        self.val = val
        self.footprint = footprint
        self.layer = layer
        self.attr = attr
        self.extra_fields = extra_fields
