# RS-232 Driver

Module Python générique pour communiquer avec un appareil via une liaison série RS-232.
Il fournit une interface asynchrone et un wrapper synchrone avec détection automatique du
port et reconnexion en cas d'erreur.

## Installation

Le module utilise `pyserial` et `pyserial-asyncio`.
Installez les dépendances via :

```bash
pip install pyserial pyserial-asyncio
```

## Utilisation synchrone

```python
from serial_rs232 import SerialClient

with SerialClient() as client:  # détecte automatiquement le port
    print(client.query("TEMP1"))
```

## Utilisation asynchrone

```python
import asyncio
from serial_rs232 import AsyncSerialClient

async def main():
    client = AsyncSerialClient()  # auto-détection du port
    await client.connect()
    print(await client.query("TEMP1"))
    await client.close()

asyncio.run(main())
```

## Options principales

- `port` : chemin du port série (auto-détection par défaut)
- `baudrate` : débit (9600 par défaut)
- `timeout` : délai de lecture en secondes
- `reconnect_delay` : délai avant tentative de reconnexion
- `eot` : caractère de fin optionnel (0x04 par défaut, peut être `None`)

## Détection de port

La fonction `detect_port()` envoie `?PING\r\n` sur chaque port et attend `PING = PONG`.
Vous pouvez personnaliser la commande de test via les paramètres `test_variable` et `expected`.

```python
from serial_rs232 import SerialClient
port = SerialClient.detect_port(test_variable="PING", expected="PONG")
```

## Format des réponses

Chaque commande `?NOM_VARIABLE\r\n` retourne : `NOM_VARIABLE = VALEUR`. Le driver
analyse automatiquement cette chaîne et renvoie un dictionnaire :

```python
{"variable": "TEMP1", "value": "23.7", "raw": "TEMP1 = 23.7"}
```

Le module reconnaît les fins de lignes `CRLF` et l'éventuel caractère `EOT`.
