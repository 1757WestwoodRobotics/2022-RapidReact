import typing


number = typing.Union[float, int]


def map_range(value: number, imin: number, imax: number, omin: number, omax: number):
    return (value - imin) * (omax - omin) / (imax - imin) + omin
