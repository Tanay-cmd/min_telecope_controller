#!/usr/bin/env python3
import datetime
from server_control.telescope_controller import calculate_julian_date, calculate_lst, ra_dec_to_alt_az, map_target_to_physical_limits, steps_to_deg

def run_tests():
    print("=== Running Coordinate Conversion Validation Tests ===")
    
    # Test Case 1: Zenith Test at J2000.0
    # Observer at Greenwich (lat=52.0, lon=0.0)
    # J2000 epoch (2000-01-01 12:00:00 UTC)
    dt1 = datetime.datetime(2000, 1, 1, 12, 0, 0, tzinfo=datetime.timezone.utc)
    jd1 = calculate_julian_date(dt1)
    lst1 = calculate_lst(longitude=0.0, dt=dt1)
    
    print(f"Test 1 - Julian Date: {jd1} (Expected: 2451545.0)")
    assert abs(jd1 - 2451545.0) < 1e-6, "JD test failed"
    
    print(f"Test 1 - LST (Greenwich): {lst1:.4f}h (Expected: ~18.6974h)")
    assert abs(lst1 - 18.69737) < 1e-3, "LST test failed"
    
    # Target is exactly on the meridian (RA = LST) and Dec = Latitude
    alt1, az1, _ = ra_dec_to_alt_az(ra=lst1, dec=52.0, lat=52.0, lon=0.0, elevation=0.0, dt=dt1)
    print(f"Test 1 - Target at Zenith: Alt={alt1:.2f}°, Az={az1:.2f}° (Expected: Alt=90.0°)")
    assert abs(alt1 - 90.0) < 1e-2, "Zenith Altitude test failed"
    
    # Test Case 2: Meridian Transit Test
    # Observer at lat=40.0, lon=-80.0
    # Target Dec = 20.0, RA = LST (on the local meridian)
    # Target should be due South (Az = 180.0) and Alt = 90 - (Lat - Dec) = 90 - 20 = 70
    dt2 = datetime.datetime(2023, 5, 15, 22, 0, 0, tzinfo=datetime.timezone.utc)
    lst2 = calculate_lst(longitude=-80.0, dt=dt2)
    alt2, az2, _ = ra_dec_to_alt_az(ra=lst2, dec=20.0, lat=40.0, lon=-80.0, elevation=0.0, dt=dt2)
    print(f"Test 2 - Meridian Transit (South): Alt={alt2:.2f}°, Az={az2:.2f}° (Expected: Alt=70.0°, Az=180.0°)")
    assert abs(alt2 - 70.0) < 1.0, "South transit altitude failed"
    assert abs(az2 - 180.0) < 1.0, "South transit azimuth failed"
    
    # Test Case 3: Target North of Zenith
    # Observer at lat=40.0, lon=-80.0
    # Target Dec = 60.0 (Dec > Lat), RA = LST
    # Target should be due North (Az = 0.0 or 360.0) and Alt = 90 - (Dec - Lat) = 90 - 20 = 70
    alt3, az3, _ = ra_dec_to_alt_az(ra=lst2, dec=60.0, lat=40.0, lon=-80.0, elevation=0.0, dt=dt2)
    print(f"Test 3 - Meridian Transit (North): Alt={alt3:.2f}°, Az={az3:.2f}° (Expected: Alt=70.0°, Az=0.0/360.0°)")
    assert abs(alt3 - 70.0) < 1.0, "North transit altitude failed"
    assert abs(az3) < 1.0 or abs(az3 - 360.0) < 1.0, "North transit azimuth failed"
    
    # Test Case 4: Rate-of-change test with non-zero seconds
    dt4_1 = datetime.datetime(2026, 5, 27, 12, 0, 0, tzinfo=datetime.timezone.utc)
    dt4_2 = dt4_1 + datetime.timedelta(seconds=2)
    lst4_1 = calculate_lst(0.0, dt4_1)
    lst4_2 = calculate_lst(0.0, dt4_2)
    lst_diff_seconds = (lst4_2 - lst4_1) * 3600.0
    print(f"Test 4 - 2s Time Delta LST change: {lst_diff_seconds:.6f}s (Expected: ~2.0055s)")
    assert abs(lst_diff_seconds - 2.00547) < 1e-4, "LST rate of change test failed"
    
    # Test Case 5: Map Target to Physical Limits (±90° Azimuth North-centered constraint)
    # Target in Northern sky (e.g. Az = 45.0, Alt = 45.0) -> No flip
    phys_az_5_1, phys_alt_5_1 = map_target_to_physical_limits(45.0, 45.0)
    print(f"Test 5.1 - No Flip (NE): Phys_Az={phys_az_5_1:.1f}°, Phys_Alt={phys_alt_5_1:.1f}° (Expected: 45.0°, 45.0°)")
    assert abs(phys_az_5_1 - 45.0) < 1e-2 and abs(phys_alt_5_1 - 45.0) < 1e-2, "Test 5.1 failed"
    
    # Target in Southern sky (e.g. Az = 135.0, Alt = 30.0) -> Flip
    phys_az_5_2, phys_alt_5_2 = map_target_to_physical_limits(135.0, 30.0)
    print(f"Test 5.2 - Flip (SE): Phys_Az={phys_az_5_2:.1f}°, Phys_Alt={phys_alt_5_2:.1f}° (Expected: -45.0°, 150.0°)")
    assert abs(phys_az_5_2 - (-45.0)) < 1e-2 and abs(phys_alt_5_2 - 150.0) < 1e-2, "Test 5.2 failed"
    
    # Test Case 6: Absolute physical degree conversion and clamping logic
    # Setup Polaris reference at 24.5 Alt, 0 Az
    ref_az = 0.0
    ref_alt = 24.5
    gear_ratio = 1.0
    
    # 5.0 absolute altitude limit: steps relative to 24.5
    # (5.0 - 24.5) * (2000 / 360) = -19.5 * 5.55556 = -108.3 steps
    limit_alt_steps = -108 # round(-108.3)
    deg_calc = steps_to_deg(limit_alt_steps, ref_alt, gear_ratio, is_azimuth=False)
    print(f"Test 6.1 - Steps to Deg (Alt): {deg_calc:.2f}° (Expected: ~5.0°)")
    assert abs(deg_calc - 5.0) < 0.2, "Steps to Deg conversion failed"
    
    print("\n[SUCCESS] All astronomical calculation validation tests passed!")

if __name__ == "__main__":
    run_tests()
