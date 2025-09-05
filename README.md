# Spirale_serial
Serial connexion for Spirale

## Installation

Ce projet dépend de [pyserial](https://pypi.org/project/pyserial/) et de [pyserial-asyncio](https://pypi.org/project/pyserial-asyncio/).
Installez-les via pip :

```bash
pip install -r requirements.txt
# ou
pip install pyserial pyserial-asyncio
```

## Utilisation

### Mode synchrone

```python
from serial_rs232 import SerialClient

# Détection automatique du port ou repli sur /dev/ttyUSB0
port = SerialClient.detect_port() or "/dev/ttyUSB0"

client = SerialClient(port, reconnect_delay=2.0)
print(client.query("TEMP1"))
client.close()
```

### Mode asynchrone

```python
import asyncio
from serial_rs232 import AsyncSerialClient, SerialClient

async def main():
    port = SerialClient.detect_port() or "/dev/ttyUSB0"
    client = AsyncSerialClient(port, reconnect_delay=2.0)
    await client.connect()
    print(await client.query("TEMP1"))
    await client.close()

asyncio.run(main())
```

### Reconnexion automatique

Le paramètre `reconnect_delay` (en secondes) permet au client de tenter une reconnexion automatique en cas de déconnexion :

```python
SerialClient(port, reconnect_delay=2.0)
AsyncSerialClient(port, reconnect_delay=2.0)
```
