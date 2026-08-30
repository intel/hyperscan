import requests

ZAPIER_WEBHOOK_URL = "https://hooks.zapier.com/hooks/catch/123456/abcdef/"

def trigger_event(event_name, data):
    payload = {
        "event": event_name,
        "data": data
    }
    response = requests.post(ZAPIER_WEBHOOK_URL, json=payload)
    response.raise_for_status()
    return response.json()

def action_create_item(item_name, item_data):
    payload = {
        "action": "create_item",
        "item_name": item_name,
        "item_data": item_data
    }
    response = requests.post(ZAPIER_WEBHOOK_URL, json=payload)
    response.raise_for_status()
    return response.json()

def action_update_item(item_id, item_data):
    payload = {
        "action": "update_item",
        "item_id": item_id,
        "item_data": item_data
    }
    response = requests.post(ZAPIER_WEBHOOK_URL, json=payload)
    response.raise_for_status()
    return response.json()

def action_delete_item(item_id):
    payload = {
        "action": "delete_item",
        "item_id": item_id
    }
    response = requests.post(ZAPIER_WEBHOOK_URL, json=payload)
    response.raise_for_status()
    return response.json()
