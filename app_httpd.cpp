#include "Arduino.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "board_config.h"
#include "Fusion.h"
#include "mbedtls/base64.h"

// ===================================================================
// Posiciones (x, y) de los 12 píxeles a muestrear.
// Origen (0,0) = esquina superior izquierda de la imagen.
// Edita estos valores según la resolución de la cámara que uses.
// ===================================================================
struct PixelPos {
  int x;
  int y;
};

static const PixelPos COLOR_SAMPLE_POINTS[12] = {
  { 50, 50 }, { 125, 50 }, { 200, 50 }, { 275, 50 },
  { 50, 125 }, { 125, 125 }, { 200, 125 }, { 275, 125 },
  { 50, 200 }, { 125, 200 }, { 200, 200 }, { 275, 200 }
};

// Intensidad del LED flash (0-255) al tomar la foto en /colors.
// 0 = apagado, 255 = máximo brillo. Cambia este valor según necesites.
static int COLOR_CAPTURE_LED_INTENSITY = 255;

// Devuelve el nombre en español de un Color
static const char *colorName(Color c) {
  switch (c) {
    case BLACK:  return "Negro";
    case WHITE:  return "Blanco";
    case BLUE:   return "Azul";
    case GREEN:  return "Verde";
    case YELLOW: return "Amarillo";
    default:     return "Desconocido";
  }
}

// Lee el color (r,g,b) del píxel (x,y) dentro de un buffer BMP de 24 bits.
// Lee el ancho y alto reales desde la cabecera del BMP (NO los acepta como
// parámetros) para evitar errores cuando frame2bmp produce un BMP con
// dimensiones distintas a fb->width / fb->height (el encoder JPEG del OV2640
// alinea a bloques MCU de 8/16 píxeles, lo que puede ampliar las dimensiones).
static bool getBmpPixel(uint8_t *buf, size_t buf_len,
                        int x, int y,
                        uint8_t &r, uint8_t &g, uint8_t &b) {
  if (!buf || buf_len < 54) return false;

  uint32_t dataOffset = buf[10] | (buf[11] << 8) | (buf[12] << 16) | (buf[13] << 24);
  int32_t  width      = (int32_t)(buf[18] | (buf[19] << 8) | (buf[20] << 16) | (buf[21] << 24));
  int32_t  height     = (int32_t)(buf[22] | (buf[23] << 8) | (buf[24] << 16) | (buf[25] << 24));
  bool     topDown    = height < 0;
  int      absHeight  = abs(height);

  if (x < 0 || y < 0 || x >= width || y >= absHeight) return false;

  int    rowSize   = ((width * 3 + 3) / 4) * 4;              // alineación a 4 bytes
  int    actualRow = topDown ? y : (absHeight - 1 - y);       // BMP bottom-up por defecto
  size_t pixOff    = dataOffset + (size_t)actualRow * rowSize + (size_t)x * 3;

  if (pixOff + 2 >= buf_len) return false;

  // BMP almacena los canales como B, G, R
  r = buf[pixOff];
  g = buf[pixOff + 1];
  b = buf[pixOff + 2];
  return true;
}

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// LED FLASH setup
#if defined(LED_GPIO_NUM)
#define CONFIG_LED_MAX_INTENSITY 255

int led_duty = 0;
bool isStreaming = false;

#endif

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}
#endif

#if defined(LED_GPIO_NUM)
void enable_led(bool en) {  // Turn LED On or Off
  int duty = en ? led_duty : 0;
  if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_GPIO_NUM, duty);
  //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
  //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
  log_i("Set LED intensity to %d", duty);
}
#endif

// Calcula el color promedio dentro de un rectángulo de píxeles de una imagen BMP de 24 bpp.
// (x, y): esquina superior izquierda del rectángulo, en coordenadas de imagen (0,0 = arriba a la izquierda)
// (w, h): ancho y alto del rectángulo a muestrear
ColorData getAverageColorBMP(uint8_t *buf, size_t buf_len, int x, int y, int w, int h) {
  ColorData avg(0, 0, 0);
  if (!buf || buf_len < 54) return avg;

  // Cabecera BMP: offset de los datos de píxeles, dimensiones y bits por píxel
  uint32_t dataOffset = buf[10] | (buf[11] << 8) | (buf[12] << 16) | (buf[13] << 24);
  int32_t width  = buf[18] | (buf[19] << 8) | (buf[20] << 16) | (buf[21] << 24);
  int32_t height = buf[22] | (buf[23] << 8) | (buf[24] << 16) | (buf[25] << 24);
  uint16_t bpp   = buf[28] | (buf[29] << 8);

  if (bpp != 24) {
    Serial.println("getAverageColorBMP: solo soporta BMP de 24 bpp");
    return avg;
  }

  bool topDown = height < 0;
  int absHeight = abs(height);

  // Cada fila de un BMP se alinea a un múltiplo de 4 bytes
  int rowStride = ((width * 3 + 3) / 4) * 4;

  // Recortamos el rango pedido a los límites reales de la imagen
  int x0 = max(0, x);
  int y0 = max(0, y);
  int x1 = min((int)width, x + w);
  int y1 = min(absHeight, y + h);

  long sumR = 0, sumG = 0, sumB = 0;
  long count = 0;

  for (int row = y0; row < y1; row++) {
    // Los BMP normalmente almacenan las filas de abajo hacia arriba
    int actualRow = topDown ? row : (absHeight - 1 - row);
    size_t rowOffset = dataOffset + (size_t)actualRow * rowStride;

    for (int col = x0; col < x1; col++) {
      size_t pixelOffset = rowOffset + (size_t)col * 3;
      if (pixelOffset + 2 >= buf_len) continue;

      // BMP guarda los canales en orden B, G, R (no R, G, B)
      uint8_t b = buf[pixelOffset];
      uint8_t g = buf[pixelOffset + 1];
      uint8_t r = buf[pixelOffset + 2];

      sumB += b;
      sumG += g;
      sumR += r;
      count++;
    }
  }

  if (count > 0) {
    avg.r = sumR / count;
    avg.g = sumG / count;
    avg.b = sumB / count;
  }

  return avg;
}

static esp_err_t bmp_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();
#endif
  fb = esp_camera_fb_get();
  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/x-windows-bmp");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  // Cast to uint32_t is safe until year 2106.
  snprintf(ts, 32, "%" PRIu32 ".%06" PRIu32, (uint32_t)fb->timestamp.tv_sec, (uint32_t)fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

  uint8_t *buf = NULL;
  size_t buf_len = 0;
  bool converted = frame2bmp(fb, &buf, &buf_len);
  esp_camera_fb_return(fb);
  if (!converted) {
    log_e("BMP Conversion failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  // Detección de color sobre el frame convertido a BMP
  Serial.printf("Total pixels %d \n", buf_len);

  // Promedia un rectángulo de 20x20 píxeles a partir de (100,100). Ajusta estos valores a tu zona de interés.
  ColorData avgColor = getAverageColorBMP(buf, buf_len, 100, 100, 20, 20);
  Serial.printf("Color promedio en rango: R=%d, G=%d, B=%d\n", avgColor.r, avgColor.g, avgColor.b);
  Color color = Fusion::rgbToColor(avgColor);
  Serial.println(color);

  res = httpd_resp_send(req, (const char *)buf, buf_len);
  free(buf);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_end = esp_timer_get_time();
#endif
  log_i("BMP: %" PRId32 "ms, %" PRIu32 "B", (int32_t)((fr_end - fr_start) / 1000), (uint32_t)buf_len);
  return res;
}

// Captura una imagen, detecta el color en los 12 puntos de COLOR_SAMPLE_POINTS y
// escribe los resultados en `results[12]`. Devuelve true si tuvo éxito.
// Usada tanto por el comando Serial2 "GO" como por el endpoint /colors.
bool captureAndClassifyColors(Color results[12]) {
#if defined(LED_GPIO_NUM)
  ledcWrite(LED_GPIO_NUM, COLOR_CAPTURE_LED_INTENSITY);
  vTaskDelay(150 / portTICK_PERIOD_MS);
#endif

  camera_fb_t *fb = esp_camera_fb_get();

#if defined(LED_GPIO_NUM)
  ledcWrite(LED_GPIO_NUM, 0);
#endif

  if (!fb) {
    log_e("captureAndClassifyColors: camera capture failed");
    return false;
  }

  uint8_t *bmp_buf = NULL;
  size_t bmp_len = 0;
  bool converted = frame2bmp(fb, &bmp_buf, &bmp_len);
  esp_camera_fb_return(fb);

  if (!converted) {
    log_e("captureAndClassifyColors: BMP conversion failed");
    return false;
  }

  for (int i = 0; i < 12; i++) {
    uint8_t r, g, b;
    if (getBmpPixel(bmp_buf, bmp_len, COLOR_SAMPLE_POINTS[i].x, COLOR_SAMPLE_POINTS[i].y, r, g, b)) {
      results[i] = Fusion::rgbToColor({r, g, b});
    } else {
      results[i] = BLACK;  // punto fuera del frame → valor de fallback
    }
  }

  free(bmp_buf);
  return true;
}

// Captura una imagen, la incrusta en la página, dibuja un marcador numerado
// sobre cada uno de los 12 puntos de COLOR_SAMPLE_POINTS, y muestra el color
// detectado en cada punto usando Fusion::rgbToColor.
static esp_err_t color_grid_handler(httpd_req_t *req) {
#if defined(LED_GPIO_NUM)
  ledcWrite(LED_GPIO_NUM, COLOR_CAPTURE_LED_INTENSITY);
  vTaskDelay(150 / portTICK_PERIOD_MS);  // dar tiempo a que el LED ilumine antes de capturar
#endif

  camera_fb_t *fb = esp_camera_fb_get();

#if defined(LED_GPIO_NUM)
  ledcWrite(LED_GPIO_NUM, 0);  // apagar el LED tras la captura
#endif

  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  int width = fb->width;
  int height = fb->height;

  // --- JPEG para mostrar la foto en el navegador ---
  uint8_t *jpg_buf = NULL;
  size_t jpg_len = 0;
  bool jpg_needs_free = false;
  if (fb->format == PIXFORMAT_JPEG) {
    jpg_buf = fb->buf;  // ya viene en JPEG, no hace falta convertir
    jpg_len = fb->len;
  } else {
    jpg_needs_free = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
  }

  // --- BMP para poder leer los píxeles individuales ---
  uint8_t *bmp_buf = NULL;
  size_t bmp_len = 0;
  bool bmp_converted = frame2bmp(fb, &bmp_buf, &bmp_len);

  esp_camera_fb_return(fb);

  if (!bmp_converted || !jpg_buf || jpg_len == 0) {
    log_e("Conversion failed");
    if (jpg_needs_free && jpg_buf) free(jpg_buf);
    if (bmp_buf) free(bmp_buf);
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  // --- Codificar el JPEG en base64 para incrustarlo directamente en el HTML ---
  size_t b64_len = 0;
  mbedtls_base64_encode(NULL, 0, &b64_len, jpg_buf, jpg_len);
  uint8_t *b64_buf = (uint8_t *)malloc(b64_len);
  String dataUri = "";
  if (b64_buf) {
    size_t actual_len = 0;
    mbedtls_base64_encode(b64_buf, b64_len, &actual_len, jpg_buf, jpg_len);
    dataUri = "data:image/jpeg;base64," + String((char *)b64_buf, actual_len);
    free(b64_buf);
  }
  if (jpg_needs_free) free(jpg_buf);

  // --- Muestrear los 12 puntos una sola vez (se reutiliza para marcadores y tarjetas) ---
  uint8_t rs[12], gs[12], bs[12];
  bool oks[12];
  Color colors[12];

  for (int i = 0; i < 12; i++) {
    int x = COLOR_SAMPLE_POINTS[i].x;
    int y = COLOR_SAMPLE_POINTS[i].y;
    oks[i] = getBmpPixel(bmp_buf, bmp_len, x, y, rs[i], gs[i], bs[i]);
    if (oks[i]) colors[i] = Fusion::rgbToColor({ rs[i], gs[i], bs[i] });
  }
  free(bmp_buf);

  // --- Construir el HTML ---
  String html = "<html><head><meta charset='UTF-8'><title>Colores detectados</title>";
  html += "<style>body{font-family:sans-serif;background:#f4f4f4;}";
  html += ".imgwrap{position:relative;display:inline-block;max-width:100%;}";
  html += ".imgwrap img{max-width:100%;height:auto;display:block;border-radius:6px;}";
  html += ".marker{position:absolute;width:22px;height:22px;border-radius:50%;";
  html += "border:2px solid #fff;box-shadow:0 0 0 1px #000;transform:translate(-50%,-50%);";
  html += "display:flex;align-items:center;justify-content:center;";
  html += "font-size:11px;font-weight:bold;color:#fff;text-shadow:0 0 2px #000;}";
  html += ".grid{display:flex;flex-wrap:wrap;margin-top:16px;}";
  html += ".pixel{width:120px;margin:8px;text-align:center;background:#fff;";
  html += "border:1px solid #ddd;border-radius:6px;padding:8px;}";
  html += ".swatch{width:60px;height:60px;border:1px solid #333;margin:0 auto 6px auto;border-radius:4px;}";
  html += "</style></head><body>";
  html += "<h2>Colores detectados (" + String(width) + "x" + String(height) + ")</h2>";

  // Imagen con marcadores superpuestos
  html += "<div class='imgwrap'>";
  html += "<img src='" + dataUri + "'>";
  for (int i = 0; i < 12; i++) {
    if (!oks[i]) continue;
    float leftPct = (COLOR_SAMPLE_POINTS[i].x * 100.0f) / width;
    float topPct = (COLOR_SAMPLE_POINTS[i].y * 100.0f) / height;
    char hex[8];
    snprintf(hex, sizeof(hex), "#%02X%02X%02X", rs[i], gs[i], bs[i]);
    html += "<div class='marker' style='left:" + String(leftPct, 2) + "%; top:" + String(topPct, 2)
            + "%; background:" + String(hex) + ";'>" + String(i + 1) + "</div>";
  }
  html += "</div>";

  // Tarjetas con el detalle de cada punto
  html += "<div class='grid'>";
  for (int i = 0; i < 12; i++) {
    int x = COLOR_SAMPLE_POINTS[i].x;
    int y = COLOR_SAMPLE_POINTS[i].y;
    html += "<div class='pixel'>";
    if (oks[i]) {
      char hex[8];
      snprintf(hex, sizeof(hex), "#%02X%02X%02X", rs[i], gs[i], bs[i]);
      html += "<div class='swatch' style='background:" + String(hex) + ";'></div>";
      html += "<div>#" + String(i + 1) + " (" + String(x) + "," + String(y) + ")</div>";
      html += "<div><b>" + String(colorName(colors[i])) + "</b></div>";
      html += "<div>RGB(" + String(rs[i]) + "," + String(gs[i]) + "," + String(bs[i]) + ")</div>";
    } else {
      html += "<div class='swatch' style='background:#ccc;'></div>";
      html += "<div>#" + String(i + 1) + " (" + String(x) + "," + String(y) + ")</div>";
      html += "<div>fuera de rango</div>";
    }
    html += "</div>";
  }
  html += "</div></body></html>";

  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, html.c_str(), html.length());
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();
#endif

#if defined(LED_GPIO_NUM)
  enable_led(true);
  vTaskDelay(150 / portTICK_PERIOD_MS);  // The LED needs to be turned on ~150ms before the call to esp_camera_fb_get()
  fb = esp_camera_fb_get();              // or it won't be visible in the frame. A better way to do this is needed.
  enable_led(false);
#else
  fb = esp_camera_fb_get();
#endif

  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  // Cast to uint32_t is safe until year 2106.
  snprintf(ts, 32, "%" PRIu32 ".%06" PRIu32, (uint32_t)fb->timestamp.tv_sec, (uint32_t)fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  size_t fb_len = 0;
#endif
  if (fb->format == PIXFORMAT_JPEG) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = fb->len;
#endif
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = jchunk.len;
#endif
  }
  esp_camera_fb_return(fb);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_end = esp_timer_get_time();
#endif
  log_i("JPG: %" PRIu32 "B %" PRId32 " ms", (uint32_t)fb_len, (int32_t)((fr_end - fr_start) / 1000));
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

#if defined(LED_GPIO_NUM)
  isStreaming = true;
  enable_led(true);
#endif

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;

    frame_time /= 1000;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
#endif
    log_i(
      "MJPG: %" PRIu32 "B %" PRId32 "ms (%.1ffps), AVG: %" PRIu32 "ms (%.1ffps)", (uint32_t)_jpg_buf_len, (int32_t)frame_time, 1000.0 / frame_time,
      avg_frame_time, 1000.0 / avg_frame_time
    );
  }

#if defined(LED_GPIO_NUM)
  isStreaming = false;
  enable_led(false);
#endif

  return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf) {
  char *buf = NULL;
  size_t buf_len = 0;

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      *obuf = buf;
      return ESP_OK;
    }
    free(buf);
  }
  httpd_resp_send_404(req);
  return ESP_FAIL;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf = NULL;
  char variable[32];
  char value[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK || httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int val = atoi(value);
  log_i("%s = %d", variable, val);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) {
      res = s->set_framesize(s, (framesize_t)val);
    }
  } else if (!strcmp(variable, "quality")) {
    res = s->set_quality(s, val);
  } else if (!strcmp(variable, "contrast")) {
    res = s->set_contrast(s, val);
  } else if (!strcmp(variable, "brightness")) {
    res = s->set_brightness(s, val);
  } else if (!strcmp(variable, "saturation")) {
    res = s->set_saturation(s, val);
  } else if (!strcmp(variable, "gainceiling")) {
    res = s->set_gainceiling(s, (gainceiling_t)val);
  } else if (!strcmp(variable, "colorbar")) {
    res = s->set_colorbar(s, val);
  } else if (!strcmp(variable, "awb")) {
    res = s->set_whitebal(s, val);
  } else if (!strcmp(variable, "agc")) {
    res = s->set_gain_ctrl(s, val);
  } else if (!strcmp(variable, "aec")) {
    res = s->set_exposure_ctrl(s, val);
  } else if (!strcmp(variable, "hmirror")) {
    res = s->set_hmirror(s, val);
  } else if (!strcmp(variable, "vflip")) {
    res = s->set_vflip(s, val);
  } else if (!strcmp(variable, "awb_gain")) {
    res = s->set_awb_gain(s, val);
  } else if (!strcmp(variable, "agc_gain")) {
    res = s->set_agc_gain(s, val);
  } else if (!strcmp(variable, "aec_value")) {
    res = s->set_aec_value(s, val);
  } else if (!strcmp(variable, "aec2")) {
    res = s->set_aec2(s, val);
  } else if (!strcmp(variable, "dcw")) {
    res = s->set_dcw(s, val);
  } else if (!strcmp(variable, "bpc")) {
    res = s->set_bpc(s, val);
  } else if (!strcmp(variable, "wpc")) {
    res = s->set_wpc(s, val);
  } else if (!strcmp(variable, "raw_gma")) {
    res = s->set_raw_gma(s, val);
  } else if (!strcmp(variable, "lenc")) {
    res = s->set_lenc(s, val);
  } else if (!strcmp(variable, "special_effect")) {
    res = s->set_special_effect(s, val);
  } else if (!strcmp(variable, "wb_mode")) {
    res = s->set_wb_mode(s, val);
  } else if (!strcmp(variable, "ae_level")) {
    res = s->set_ae_level(s, val);
  }
#if defined(LED_GPIO_NUM)
  else if (!strcmp(variable, "led_intensity")) {
    led_duty = val;
    if (isStreaming) {
      enable_led(true);
    }
  }
#endif
  else {
    log_i("Unknown command: %s", variable);
    res = -1;
  }

  if (res < 0) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static int print_reg(char *p, char *end, sensor_t *s, uint16_t reg, uint32_t mask) {
  return snprintf(p, end - p, "\"0x%04x\":%d,", reg, s->get_reg(s, reg, mask));
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  char *end = json_response + sizeof(json_response);
  *p++ = '{';

  if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID) {
    for (int reg = 0x3400; reg < 0x3406; reg += 2) {
      p += print_reg(p, end, s, reg, 0xFFF);  //12 bit
    }
    p += print_reg(p, end, s, 0x3406, 0xFF);

    p += print_reg(p, end, s, 0x3500, 0xFFFF0);  //16 bit
    p += print_reg(p, end, s, 0x3503, 0xFF);
    p += print_reg(p, end, s, 0x350a, 0x3FF);   //10 bit
    p += print_reg(p, end, s, 0x350c, 0xFFFF);  //16 bit

    for (int reg = 0x5480; reg <= 0x5490; reg++) {
      p += print_reg(p, end, s, reg, 0xFF);
    }

    for (int reg = 0x5380; reg <= 0x538b; reg++) {
      p += print_reg(p, end, s, reg, 0xFF);
    }

    for (int reg = 0x5580; reg < 0x558a; reg++) {
      p += print_reg(p, end, s, reg, 0xFF);
    }
    p += print_reg(p, end, s, 0x558a, 0x1FF);  //9 bit
  } else if (s->id.PID == OV2640_PID) {
    p += print_reg(p, end, s, 0xd3, 0xFF);
    p += print_reg(p, end, s, 0x111, 0xFF);
    p += print_reg(p, end, s, 0x132, 0xFF);
  }

  p += snprintf(p, end - p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
  p += snprintf(p, end - p, "\"pixformat\":%u,", s->pixformat);
  p += snprintf(p, end - p, "\"framesize\":%u,", s->status.framesize);
  p += snprintf(p, end - p, "\"quality\":%u,", s->status.quality);
  p += snprintf(p, end - p, "\"brightness\":%d,", s->status.brightness);
  p += snprintf(p, end - p, "\"contrast\":%d,", s->status.contrast);
  p += snprintf(p, end - p, "\"saturation\":%d,", s->status.saturation);
  p += snprintf(p, end - p, "\"sharpness\":%d,", s->status.sharpness);
  p += snprintf(p, end - p, "\"special_effect\":%u,", s->status.special_effect);
  p += snprintf(p, end - p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += snprintf(p, end - p, "\"awb\":%u,", s->status.awb);
  p += snprintf(p, end - p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += snprintf(p, end - p, "\"aec\":%u,", s->status.aec);
  p += snprintf(p, end - p, "\"aec2\":%u,", s->status.aec2);
  p += snprintf(p, end - p, "\"ae_level\":%d,", s->status.ae_level);
  p += snprintf(p, end - p, "\"aec_value\":%u,", s->status.aec_value);
  p += snprintf(p, end - p, "\"agc\":%u,", s->status.agc);
  p += snprintf(p, end - p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += snprintf(p, end - p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += snprintf(p, end - p, "\"bpc\":%u,", s->status.bpc);
  p += snprintf(p, end - p, "\"wpc\":%u,", s->status.wpc);
  p += snprintf(p, end - p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += snprintf(p, end - p, "\"lenc\":%u,", s->status.lenc);
  p += snprintf(p, end - p, "\"hmirror\":%u,", s->status.hmirror);
  p += snprintf(p, end - p, "\"vflip\":%u,", s->status.vflip);
  p += snprintf(p, end - p, "\"dcw\":%u,", s->status.dcw);
  p += snprintf(p, end - p, "\"colorbar\":%u", s->status.colorbar);
#if defined(LED_GPIO_NUM)
  p += snprintf(p, end - p, ",\"led_intensity\":%u", led_duty);
#else
  p += snprintf(p, end - p, ",\"led_intensity\":%d", -1);
#endif
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t xclk_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _xclk[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "xclk", _xclk, sizeof(_xclk)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int xclk = atoi(_xclk);
  log_i("Set XCLK: %d MHz", xclk);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t reg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];
  char _val[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK
      || httpd_query_key_value(buf, "val", _val, sizeof(_val)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  int val = atoi(_val);
  log_i("Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_reg(s, reg, mask, val);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t greg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->get_reg(s, reg, mask);
  if (res < 0) {
    return httpd_resp_send_500(req);
  }
  log_i("Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

  char buffer[20];
  const char *val = itoa(res, buffer, 10);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, val, strlen(val));
}

static int parse_get_var(char *buf, const char *key, int def) {
  char _int[16];
  if (httpd_query_key_value(buf, key, _int, sizeof(_int)) != ESP_OK) {
    return def;
  }
  return atoi(_int);
}

static esp_err_t pll_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int bypass = parse_get_var(buf, "bypass", 0);
  int mul = parse_get_var(buf, "mul", 0);
  int sys = parse_get_var(buf, "sys", 0);
  int root = parse_get_var(buf, "root", 0);
  int pre = parse_get_var(buf, "pre", 0);
  int seld5 = parse_get_var(buf, "seld5", 0);
  int pclken = parse_get_var(buf, "pclken", 0);
  int pclk = parse_get_var(buf, "pclk", 0);
  free(buf);

  log_i("Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", bypass, mul, sys, root, pre, seld5, pclken, pclk);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t win_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int startX = parse_get_var(buf, "sx", 0);
  int startY = parse_get_var(buf, "sy", 0);
  int endX = parse_get_var(buf, "ex", 0);
  int endY = parse_get_var(buf, "ey", 0);
  int offsetX = parse_get_var(buf, "offx", 0);
  int offsetY = parse_get_var(buf, "offy", 0);
  int totalX = parse_get_var(buf, "tx", 0);
  int totalY = parse_get_var(buf, "ty", 0);  // codespell:ignore totaly
  int outputX = parse_get_var(buf, "ox", 0);
  int outputY = parse_get_var(buf, "oy", 0);
  bool scale = parse_get_var(buf, "scale", 0) == 1;
  bool binning = parse_get_var(buf, "binning", 0) == 1;
  free(buf);

  log_i(
    "Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", startX, startY, endX, endY, offsetX, offsetY,
    totalX, totalY, outputX, outputY, scale, binning  // codespell:ignore totaly
  );
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);  // codespell:ignore totaly
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t bmp_uri = {
    .uri = "/bmp",
    .method = HTTP_GET,
    .handler = bmp_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t colors_uri = {
    .uri = "/colors",
    .method = HTTP_GET,
    .handler = color_grid_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t xclk_uri = {
    .uri = "/xclk",
    .method = HTTP_GET,
    .handler = xclk_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t reg_uri = {
    .uri = "/reg",
    .method = HTTP_GET,
    .handler = reg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t greg_uri = {
    .uri = "/greg",
    .method = HTTP_GET,
    .handler = greg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t pll_uri = {
    .uri = "/pll",
    .method = HTTP_GET,
    .handler = pll_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t win_uri = {
    .uri = "/resolution",
    .method = HTTP_GET,
    .handler = win_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  ra_filter_init(&ra_filter, 20);

  log_i("Starting web server on port: '%u'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &bmp_uri);
    httpd_register_uri_handler(camera_httpd, &colors_uri);

    httpd_register_uri_handler(camera_httpd, &xclk_uri);
    httpd_register_uri_handler(camera_httpd, &reg_uri);
    httpd_register_uri_handler(camera_httpd, &greg_uri);
    httpd_register_uri_handler(camera_httpd, &pll_uri);
    httpd_register_uri_handler(camera_httpd, &win_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  log_i("Starting stream server on port: '%u'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setupLedFlash() {
#if defined(LED_GPIO_NUM)
  ledcAttach(LED_GPIO_NUM, 5000, 8);
#else
  log_i("LED flash is disabled -> LED_GPIO_NUM undefined");
#endif
}
