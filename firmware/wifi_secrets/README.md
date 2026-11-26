# wifi_secrets

Fuente única de las credenciales Wi-Fi que usan todos los sketches en `firmware/`.
Cambie la red aquí una sola vez en lugar de editar cada sketch.

Arduino copia el sketch a una carpeta temporal de compilación antes de compilar, por lo que un
`#include "../secrets.h"` relativo no funciona. En su lugar, esta carpeta es una biblioteca de Arduino
y cada sketch usa `#include <wifi_secrets.h>`.

## Configuración (una vez por computadora)

1. Copie `wifi_secrets.example.h` a `src/wifi_secrets.h` y defina `WIFI_SSID` y
   `WIFI_PASSWORD`. git ignora `src/wifi_secrets.h`.
2. Enlace esta carpeta dentro de la carpeta de bibliotecas de Arduino (no se requieren permisos de administrador).
   Verifique la ruta del sketchbook en Arduino IDE en Archivo > Preferencias.

   ```powershell
   New-Item -ItemType Junction -Path "$HOME\OneDrive\Documentos\Arduino\libraries\wifi_secrets" -Target "<repo>\firmware\wifi_secrets"
   ```

3. Reinicie Arduino IDE.

El ESP32 solo se conecta a redes de 2.4 GHz. En un hotspot móvil de Windows, configure la banda
en 2.4 GHz.
