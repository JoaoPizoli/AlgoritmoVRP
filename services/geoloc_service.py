import googlemaps

gmaps = googlemaps.Client(key="AIzaSyC8moNSB7lRDY3FNJ-Lk0oeJ7uP82kp1-0")


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
