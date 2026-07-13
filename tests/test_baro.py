"""Model test for flight-controller/App/bmp580.c baro_altitude_m().

Mirrors the temperature-compensated ISA altitude as committed in bmp580.c:
  isa   = 44330 * (1 - (p/p0)^0.190295)
  scale = T_pad_K / ISA_T0_K, when 200 < T_pad_K < 360, else 1.0
  alt   = isa * scale
T_pad is latched in baro_zero() (measured_temp_c + 273.15). Unset/implausible
T_pad falls back to plain ISA (scale 1.0).

There is no host C compiler, so this port stands in for the C. If
bmp580.c baro_altitude_m() / ISA_T0_K change, update this in lockstep.
"""

import unittest

ISA_T0_K = 288.15   # bmp580.c ISA_T0_K


def isa_altitude(press_pa, p0_pa=101325.0):
    return 44330.0 * (1.0 - (press_pa / p0_pa) ** 0.190295)


def baro_altitude_m(press_pa, tpad_k, p0_pa=101325.0):
    """Port of bmp580.c baro_altitude_m(). tpad_k=0 (or absurd) => plain ISA."""
    if press_pa <= 0.0 or p0_pa <= 0.0:
        return 0.0
    isa = isa_altitude(press_pa, p0_pa)
    scale = 1.0
    if 200.0 < tpad_k < 360.0:
        scale = tpad_k / ISA_T0_K
    return isa * scale


class TestBaroTempComp(unittest.TestCase):
    P = 90000.0   # ~1000 m of AGL at standard p0

    def test_standard_temp_equals_isa(self):
        self.assertAlmostEqual(
            baro_altitude_m(self.P, ISA_T0_K), isa_altitude(self.P), places=6)

    def test_cold_smaller_hot_larger_by_exact_ratio(self):
        isa = isa_altitude(self.P)
        cold = 273.15  # 0 degC
        hot = 313.15   # 40 degC
        self.assertAlmostEqual(
            baro_altitude_m(self.P, cold), isa * (cold / ISA_T0_K), places=6)
        self.assertAlmostEqual(
            baro_altitude_m(self.P, hot), isa * (hot / ISA_T0_K), places=6)
        self.assertLess(baro_altitude_m(self.P, cold), isa)   # colder -> smaller
        self.assertGreater(baro_altitude_m(self.P, hot), isa)  # hotter -> larger

    def test_unset_and_absurd_fall_back_to_isa(self):
        isa = isa_altitude(self.P)
        for tpad in (0.0, -10.0, 50.0, 1000.0):   # unset / <=0 / out of range
            self.assertAlmostEqual(
                baro_altitude_m(self.P, tpad), isa, places=6)


if __name__ == "__main__":
    unittest.main()
