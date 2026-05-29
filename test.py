from datetime import datetime
from astropy.coordinates import SkyCoord, EarthLocation, AltAz
import astropy.units as u
from astropy.time import Time

def locate_zeta_persei(lat_deg, lon_deg, height_m=0):
    # 1. Parse the RA and Dec from the image using correct sexagesimal strings
    ra_str = "05h42m04.6s"
    dec_str = "-01d55m45.06s" 
    
    target = SkyCoord(ra=ra_str, dec=dec_str, frame='icrs')
    
    # 2. Grab your PC's current time directly in UTC
    utc_now = datetime.utcnow()
    
    # Explicitly tell Astropy this datetime object is in the UTC scale
    observation_time = Time(utc_now, scale='utc') 
    
    # 3. Define the observer's location
    location = EarthLocation(lat=lat_deg * u.deg, lon=lon_deg * u.deg, height=height_m * u.m)
    
    # 4. Set up the transformation frame
    alt_az_frame = AltAz(obstime=observation_time, location=location)
    
    # 5. Transform coordinates
    target_altaz = target.transform_to(alt_az_frame)
    
    # Print results neatly
    print("--- Target Located ---")
    print(f"Object:         Zeta Persei")
    print(f"Observation Time (UTC):   {utc_now.strftime('%Y-%m-%d %H:%M:%S')} UTC")
    print(f"Altitude (Alt):           {target_altaz.alt.deg:.4f}°")
    print(f"Azimuth (Az):             {target_altaz.az.deg:.4f}°")
    
    if target_altaz.alt.deg > 0:
        print("Status:                   Above the horizon (Visible if clear!)")
    else:
        print("Status:                   Below the horizon (Not currently visible)")

# --- Configure Your Location Here ---
MY_LATITUDE = 24.16       
MY_LONGITUDE = 72.78      
MY_ELEVATION = 1680       

locate_zeta_persei(lat_deg=MY_LATITUDE, lon_deg=MY_LONGITUDE, height_m=MY_ELEVATION)