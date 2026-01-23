import googlemaps
from dotenv import load_dotenv
load_dotenv()
import os

gmaps = googlemaps.Client(os.getenv("GOOGLE_API"))


def get_coordinates(rua: str, cidade: str) -> dict:
    """
    Obtém latitude e longitude de um endereço.
    
    Args:
        rua: Nome da rua com número
        cidade: Nome da cidade
    
    Returns:
        Dict com endereco, lat e lng ou None se não encontrado
    """
    endereco = f"{rua}, {cidade}"
    
    resultado = gmaps.geocode(endereco)
    
    if resultado:
        location = resultado[0]['geometry']['location']
        return {
            "endereco": endereco,
            "lat": location['lat'],
            "lng": location['lng']
        }
    
    return None
