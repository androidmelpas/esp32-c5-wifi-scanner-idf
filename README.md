# Analizador pasivo Wi-Fi / BLE / Zigbee para ESP32-C5 (ESP-IDF)

Proyecto ESP-IDF que permite elegir Wi-Fi, BLE o Zigbee al arrancar y emite eventos en CSV. El firmware funciona en modo pasivo: no se conecta a redes ni transmite tramas de análisis.

Al arrancar, el selector espera 10 segundos; si no recibe respuesta, elige Wi-Fi. En ese modo se solicita un canal Wi-Fi de 2,4 o 5 GHz y, tras otros 10 segundos sin respuesta, se usa el canal 100.

La salida contiene direcciones MAC 1-4 y las direcciones SA/DA/TA/RA/BSSID cuando se pueden determinar, tipo/subtipo, SSID anunciado en beacons y respuestas de probe, bits To DS/From DS, flags, duración, secuencia/fragmento, QoS/HTC, RSSI, nivel de ruido, SNR, canal y metadatos PHY expuestos por ESP-IDF. `frame_length_bytes` incluye el FCS. `rx_timestamp_us` es el reloj local de recepción del ESP32-C5; no es TSFT. `phy_siga1_hex` y `phy_siga2_hex` son campos PHY sin decodificar. `rx_state` es el estado bruto del controlador, no un diagnóstico FCS detallado.

En Wi-Fi, para identificar el AP, el firmware lee únicamente el SSID IE de beacons y respuestas de probe; no extrae SSID de probes de clientes ni decodifica otros IEs. El SSID aparece entre comillas en CSV; bytes no imprimibles/no ASCII se representan como `\xHH`. No exporta el payload Wi-Fi ni lee RSN, WPS, EAPOL/TLS, PNL u otros campos de cuerpos de trama. Los campos que no aplican a un tipo de trama se dejan vacíos o como `NA`.

El controlador no expone en este formato valores decodificados para todos los campos PHY pedidos (por ejemplo MCS/NSS, GI, FEC, beamforming, RU o BSS Color); se conservan los campos SIG recibidos sin interpretarlos. Los códigos de estado/motivo de gestión también están en el cuerpo de esas tramas y no se leen.

La cola de salida es limitada. Si llegan más tramas de las que puede transmitir la consola serie a 921600 baudios, `dropped_frames_total` aumenta y señala cuántas filas se descartaron; por tanto, no se garantiza registrar cada trama en canales muy ocupados. Configura el monitor serie también a 921600 baudios.

## Cabeceras CSV por modo

Cada arranque imprime solo la cabecera del modo seleccionado; las filas siguientes corresponden a esa cabecera. Los tres formatos son distintos y no se usa la cabecera Wi-Fi para BLE ni Zigbee.

**Wi-Fi**

```csv
rx_timestamp_us,channel,channel_frequency_mhz,secondary_channel,rssi_dbm,noise_floor_dbm,snr_db,band,phy_format,phy_siga1_hex,phy_siga2_hex,phy_rate_field,frame_length_bytes,rx_state,protocol_version,frame_type,subtype,frame_type_id,subtype_id,ssid,to_ds,from_ds,addr1,addr2,addr3,addr4,bssid,sa,da,ta,ra,addr2_oui,addr2_ul_bit,addr2_ig_bit,duration_id,aid,retry,more_fragments,power_management,more_data,protected,order,sequence_number,fragment_number,qos_control_hex,ht_control_hex,dropped_frames_total
```

**BLE** (anuncios pasivos; no incluye los bytes anunciados)

```csv
timestamp_us,advertisement_event_type_id,advertisement_event_type,address_type_id,address_type,address,rssi_dbm,advertisement_data_length,dropped_reports_total
```

`advertisement_event_type` identifica el tipo de anuncio; `address_type` indica si la dirección es pública o aleatoria; `address` y `rssi_dbm` identifican al emisor y la señal recibida; `advertisement_data_length` da la longitud de los datos sin exportar su contenido.

**Zigbee / IEEE 802.15.4** (solo metadatos y cabecera MAC)

```csv
timestamp_us,channel,rssi_dbm,lqi,frame_length_bytes,frame_type,frame_type_id,frame_version,security_enabled,frame_pending,ack_request,pan_id_compression,destination_address_mode,source_address_mode,destination_pan_id_hex,destination_address,source_pan_id_hex,source_address,sequence_number,dropped_frames_total,receive_handler_errors_total
```

`frame_type` clasifica la trama; los campos `destination_*` y `source_*` muestran PAN y direcciones cuando la cabecera permite interpretarlos. `security_enabled`, `frame_pending`, `ack_request` y `pan_id_compression` son flags de la cabecera MAC.

## BLE y Zigbee

- **BLE:** escaneo pasivo de anuncios; el CSV registra tipo de anuncio, tipo y dirección del emisor, RSSI y longitud de los datos anunciados, pero no exporta dichos datos. No captura tráfico de conexiones BLE ni envía solicitudes activas, así que no obtiene respuestas de escaneo que no se transmitan espontáneamente.
- **Zigbee / IEEE 802.15.4:** recibe tramas válidas en modo promiscuo y registra RSSI, LQI, tipo/flags de la cabecera MAC, PAN ID y direcciones cuando el formato se puede interpretar, sin exportar el payload. Salta por los canales 11–26 cada 100 ms. El ESP32-C5 escucha un canal cada vez, así que se pueden perder tramas durante el salto y no equivale a capturar simultáneamente todos los canales.
- En ambos modos, `dropped_reports_total` o `dropped_frames_total` informa de eventos descartados por saturación de la cola de salida. Zigbee añade `receive_handler_errors_total` para errores al devolver el buffer al controlador.
- Las direcciones BLE y PAN/MAC de Zigbee pueden identificar dispositivos. Guarda y comparte los CSV con cuidado.

## Requisitos

- ESP-IDF con soporte para ESP32-C5 y el entorno de compilación configurado.
- Placa ESP32-C5 conectada por USB.
- Consola serie a 921600 baudios.
- Partición de aplicación única de 1,5 MB para que quepa el firmware con NimBLE; esta configuración no ofrece OTA.

Los canales disponibles en el menú son `1–14` en 2,4 GHz y `36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165, 169, 173, 177` en 5 GHz. El campo `channel_frequency_mhz` refleja la frecuencia de la banda elegida (canal 14: 2484 MHz). El soporte efectivo depende del país, la placa y la versión de ESP-IDF; el canal 14 puede estar restringido y los canales DFS también dependen de la normativa local.

## Compilar y cargar desde la terminal ESP-IDF

Abre la terminal ESP-IDF en esta carpeta y ejecuta:

```text
idf.py set-target esp32c5
idf.py build
idf.py -p COMx flash monitor
```

Reemplaza `COMx` por el puerto de la placa en Windows (por ejemplo, `COM5`). En Linux/macOS, usa el dispositivo serie correspondiente.

Al arrancar, elige `1` (Wi-Fi), `2` (BLE) o `3` (Zigbee). En Wi-Fi, introduce un canal listado de 2,4 o 5 GHz y pulsa Enter. Después del encabezado CSV, cada fila describe un evento recibido. Para salir del monitor serie, pulsa `Ctrl+]`.

## Abrir en VS Code

Instala la extensión **Espressif IDF** y configura ESP-IDF cuando la extensión lo solicite. Abre esta carpeta como proyecto, selecciona `ESP32-C5` como objetivo, elige el puerto serie y usa los comandos de compilación, carga y monitor de la extensión.
