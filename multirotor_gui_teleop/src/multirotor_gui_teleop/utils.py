def remap(x: float, a: float, b: float, c: float, d: float) -> float:
    """ xを[a, b]の範囲から[c, d]の範囲に投影する． """
    return (c * (b - x) + d * (x - a)) / (b - a)
