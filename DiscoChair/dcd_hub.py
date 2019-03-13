import requests

def dcd_hub_status():
    """
    return the dcd hub status:
    - 0: connection succesful
    - 1L could not reach the dcd
    """
    uri = "https://dwd.tudelft.nl/api/health"
    try:
        json_result = requests.get(uri).json()
        if json_result["status"] is 0:
            return 0
        return 1
    except Exception as e:
        print(str(e))
        return 1
        
