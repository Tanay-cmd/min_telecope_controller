import urllib.request
import urllib.parse
import json

def resolve_target_via_simbad(name: str):
    """
    Resolve target name to coordinates using the dockerized simbad2k service.
    """
    encoded_name = urllib.parse.quote(name)
    url = f"http://localhost:5000/{encoded_name}?target_type=sidereal"
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'MinTelescopeController/1.0'})
        with urllib.request.urlopen(req, timeout=5) as response:
            data = json.loads(response.read().decode())
            if isinstance(data, dict) and 'error' not in data:
                return data
            # Fallback to non-sidereal if sidereal query failed or returned no match
            url_ns = f"http://localhost:5000/{encoded_name}?target_type=non_sidereal"
            req_ns = urllib.request.Request(url_ns, headers={'User-Agent': 'MinTelescopeController/1.0'})
            with urllib.request.urlopen(req_ns, timeout=5) as response_ns:
                data_ns = json.loads(response_ns.read().decode())
                if isinstance(data_ns, dict) and 'error' not in data_ns:
                    return data_ns
    except Exception as e:
        print(f"[WARNING] Simbad resolution error: {e}")
    return None
