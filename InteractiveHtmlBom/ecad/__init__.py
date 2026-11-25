def get_kicad_parser(file_name, config, logger, kicad=None, board=None):
    """KiCad boards are read from a running KiCad over the IPC API.

    If kicad/board are not given, connects to KiCad and checks that
    file_name (if any) is the board currently open in the PCB editor.
    """
    from .kicad_ipc import KiCadIPCParser, connect_to_kicad
    if board is None:
        kicad, board, file_name = connect_to_kicad(file_name)
    return KiCadIPCParser(file_name, config, logger, kicad, board)
