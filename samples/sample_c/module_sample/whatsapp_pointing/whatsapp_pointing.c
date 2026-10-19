/**
 ********************************************************************
 * @file    whatsapp_pointing.c
 * @brief   On the C3-mapped widget button, read where the Matrice 4T
 *          camera laser is pointing and send that position to a
 *          WhatsApp group through the WhatsApp Cloud API.
 *
 * @copyright (c) 2026 DJI. All rights reserved.
 *********************************************************************
 */

#include "whatsapp_pointing.h"

#include <dji_logger.h>
#include <dji_camera_manager.h>
#include <dji_fc_subscription.h>
#include <dji_widget.h>
#include <dji_platform.h>

#include <ctype.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define WHATSAPP_CFG_PATH_PRIMARY     "/open_app/whatsapp_pointing.conf"
#define WHATSAPP_CFG_PATH_CWD         "whatsapp_pointing.conf"
#define WHATSAPP_TEXT_MAX             512
#define WHATSAPP_TOKEN_MAX            512
#define WHATSAPP_ID_MAX               128
#define WHATSAPP_JSON_MAX             1024
#define WHATSAPP_CMD_MAX              768
#define WHATSAPP_RESP_MAX             256

typedef struct {
    char accessToken[WHATSAPP_TOKEN_MAX];
    char phoneNumberId[WHATSAPP_ID_MAX];
    char groupId[WHATSAPP_ID_MAX];
    char graphVersion[32];
} T_WhatsappConfig;

static pthread_mutex_t s_sendLock = PTHREAD_MUTEX_INITIALIZER;
static bool s_sendBusy = false;
static bool s_cameraReady = false;
static bool s_gpsReady = false;

static void Whatsapp_Show(const char *text)
{
    USER_LOG_INFO("%s", text);
    DjiWidgetFloatingWindow_ShowMessage(text);
}

static int Whatsapp_IsDigits(const char *text)
{
    size_t i;

    if (text == NULL || text[0] == '\0') {
        return 0;
    }
    for (i = 0; text[i] != '\0'; ++i) {
        if (!isdigit((unsigned char) text[i])) {
            return 0;
        }
    }
    return 1;
}

static int Whatsapp_IsSafeId(const char *text)
{
    size_t i;

    if (text == NULL || text[0] == '\0') {
        return 0;
    }
    for (i = 0; text[i] != '\0'; ++i) {
        unsigned char c = (unsigned char) text[i];
        if (!(isalnum(c) || c == '@' || c == '.' || c == '_' || c == '-')) {
            return 0;
        }
    }
    return 1;
}

static int Whatsapp_IsToken(const char *text)
{
    size_t i;

    if (text == NULL || strlen(text) < 8) {
        return 0;
    }
    for (i = 0; text[i] != '\0'; ++i) {
        unsigned char c = (unsigned char) text[i];
        if (c < 0x21 || c > 0x7e) {
            return 0;
        }
    }
    return 1;
}

static int Whatsapp_IsGraphVersion(const char *text)
{
    size_t i;

    if (text == NULL || text[0] != 'v') {
        return 0;
    }
    for (i = 1; text[i] != '\0'; ++i) {
        unsigned char c = (unsigned char) text[i];
        if (!(isdigit(c) || c == '.')) {
            return 0;
        }
    }
    return i > 1;
}

static void Whatsapp_Trim(char *text)
{
    size_t len;
    char *start = text;

    if (text == NULL) {
        return;
    }
    while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') {
        ++start;
    }
    if (start != text) {
        memmove(text, start, strlen(start) + 1);
    }
    len = strlen(text);
    while (len > 0 && (text[len - 1] == ' ' || text[len - 1] == '\t' ||
                       text[len - 1] == '\r' || text[len - 1] == '\n')) {
        text[--len] = '\0';
    }
}

static void Whatsapp_ApplyPair(T_WhatsappConfig *cfg, const char *key, const char *value)
{
    if (strcmp(key, "access_token") == 0) {
        snprintf(cfg->accessToken, sizeof(cfg->accessToken), "%s", value);
    } else if (strcmp(key, "phone_number_id") == 0) {
        snprintf(cfg->phoneNumberId, sizeof(cfg->phoneNumberId), "%s", value);
    } else if (strcmp(key, "group_id") == 0) {
        snprintf(cfg->groupId, sizeof(cfg->groupId), "%s", value);
    } else if (strcmp(key, "graph_version") == 0) {
        snprintf(cfg->graphVersion, sizeof(cfg->graphVersion), "%s", value);
    }
}

static int Whatsapp_LoadFile(T_WhatsappConfig *cfg, const char *path)
{
    FILE *file;
    char line[WHATSAPP_TOKEN_MAX + WHATSAPP_ID_MAX];

    file = fopen(path, "r");
    if (file == NULL) {
        return 0;
    }

    while (fgets(line, sizeof(line), file) != NULL) {
        char *eq;

        Whatsapp_Trim(line);
        if (line[0] == '\0' || line[0] == '#') {
            continue;
        }
        eq = strchr(line, '=');
        if (eq == NULL) {
            continue;
        }
        *eq = '\0';
        Whatsapp_Trim(line);
        Whatsapp_Trim(eq + 1);
        Whatsapp_ApplyPair(cfg, line, eq + 1);
    }
    fclose(file);
    USER_LOG_INFO("Loaded WhatsApp config from %s", path);
    return 1;
}

static T_DjiReturnCode Whatsapp_LoadConfig(T_WhatsappConfig *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    snprintf(cfg->graphVersion, sizeof(cfg->graphVersion), "v21.0");

    if (!Whatsapp_LoadFile(cfg, WHATSAPP_CFG_PATH_PRIMARY)) {
        Whatsapp_LoadFile(cfg, WHATSAPP_CFG_PATH_CWD);
    }

    if (!Whatsapp_IsToken(cfg->accessToken) || !Whatsapp_IsDigits(cfg->phoneNumberId) ||
        !Whatsapp_IsSafeId(cfg->groupId) || !Whatsapp_IsGraphVersion(cfg->graphVersion) ||
        strcmp(cfg->accessToken, "your_access_token") == 0 ||
        strcmp(cfg->groupId, "your_group_id") == 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static int Whatsapp_JsonEscape(const char *in, char *out, size_t outLen)
{
    size_t used = 0;

    if (in == NULL || out == NULL || outLen == 0) {
        return -1;
    }
    while (*in != '\0') {
        const char *add = NULL;
        char one[2];
        size_t addLen;

        switch (*in) {
            case '\\':
                add = "\\\\";
                break;
            case '"':
                add = "\\\"";
                break;
            case '\n':
                add = "\\n";
                break;
            case '\r':
                add = "\\r";
                break;
            default:
                one[0] = *in;
                one[1] = '\0';
                add = one;
                break;
        }
        addLen = strlen(add);
        if (used + addLen + 1 > outLen) {
            return -1;
        }
        memcpy(out + used, add, addLen);
        used += addLen;
        ++in;
    }
    out[used] = '\0';
    return 0;
}

static T_DjiReturnCode Whatsapp_ReadLaser(double *latDeg, double *lonDeg, double *altM, double *rangeM)
{
    T_DjiReturnCode returnCode;
    T_DjiCameraManagerLaserRangingInfo rangingInfo = {0};

    if (!s_cameraReady) {
        returnCode = DjiCameraManager_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Camera manager init failed, stat = 0x%08X", returnCode);
            return returnCode;
        }
        s_cameraReady = true;
    }

    returnCode = DjiCameraManager_GetLaserRangingInfo(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &rangingInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Laser ranging read failed, stat = 0x%08X", returnCode);
        return returnCode;
    }
    if (rangingInfo.exception != 0 ||
        rangingInfo.latitude < -90.0 || rangingInfo.latitude > 90.0 ||
        rangingInfo.longitude < -180.0 || rangingInfo.longitude > 180.0 ||
        (rangingInfo.latitude == 0.0 && rangingInfo.longitude == 0.0)) {
        USER_LOG_WARN("Laser ranging has no valid point, exception=%u enable=%d",
                      rangingInfo.exception, rangingInfo.enable_lidar);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    *latDeg = rangingInfo.latitude;
    *lonDeg = rangingInfo.longitude;
    *altM = (double) rangingInfo.altitude / 10.0;
    *rangeM = (double) rangingInfo.distance / 10.0;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Whatsapp_ReadAircraftGps(double *latDeg, double *lonDeg, double *altM)
{
    T_DjiReturnCode returnCode;
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiDataTimestamp timestamp = {0};

    if (!s_gpsReady) {
        returnCode = DjiFcSubscription_Init();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("FC subscription init failed, stat = 0x%08X", returnCode);
            return returnCode;
        }
        returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                      NULL);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("GPS subscribe failed, stat = 0x%08X", returnCode);
            return returnCode;
        }
        s_gpsReady = true;
        sleep(1);
    }

    returnCode = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                         (uint8_t *) &gpsPosition,
                                                         sizeof(gpsPosition),
                                                         &timestamp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("GPS read failed, stat = 0x%08X", returnCode);
        return returnCode;
    }

    /* x = longitude, y = latitude, unit deg * 10^-7. z = altitude, unit mm. */
    *lonDeg = (double) gpsPosition.x / 10000000.0;
    *latDeg = (double) gpsPosition.y / 10000000.0;
    *altM = (double) gpsPosition.z / 1000.0;
    if (*latDeg < -90.0 || *latDeg > 90.0 || *lonDeg < -180.0 || *lonDeg > 180.0 ||
        (*latDeg == 0.0 && *lonDeg == 0.0)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Whatsapp_Post(const T_WhatsappConfig *cfg, const char *message)
{
    char escaped[WHATSAPP_JSON_MAX];
    char json[WHATSAPP_JSON_MAX];
    char headerPath[] = "/tmp/psdk_wa_hdr_XXXXXX";
    char bodyPath[] = "/tmp/psdk_wa_body_XXXXXX";
    char cmd[WHATSAPP_CMD_MAX];
    char response[WHATSAPP_RESP_MAX];
    int headerFd;
    int bodyFd;
    FILE *headerFile;
    FILE *bodyFile;
    FILE *curl;
    int status;

    if (Whatsapp_JsonEscape(message, escaped, sizeof(escaped)) != 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    if (snprintf(json, sizeof(json),
                 "{\"messaging_product\":\"whatsapp\",\"recipient_type\":\"group\",\"to\":\"%s\","
                 "\"type\":\"text\",\"text\":{\"body\":\"%s\"}}",
                 cfg->groupId, escaped) >= (int) sizeof(json)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    headerFd = mkstemp(headerPath);
    bodyFd = mkstemp(bodyPath);
    if (headerFd < 0 || bodyFd < 0) {
        USER_LOG_ERROR("Create temp files failed, errno=%d", errno);
        if (headerFd >= 0) {
            close(headerFd);
            unlink(headerPath);
        }
        if (bodyFd >= 0) {
            close(bodyFd);
            unlink(bodyPath);
        }
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    headerFile = fdopen(headerFd, "w");
    bodyFile = fdopen(bodyFd, "w");
    if (headerFile == NULL || bodyFile == NULL) {
        USER_LOG_ERROR("Open temp files failed");
        if (headerFile != NULL) {
            fclose(headerFile);
        } else {
            close(headerFd);
        }
        if (bodyFile != NULL) {
            fclose(bodyFile);
        } else {
            close(bodyFd);
        }
        unlink(headerPath);
        unlink(bodyPath);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    fprintf(headerFile, "Authorization: Bearer %s\n", cfg->accessToken);
    fputs(json, bodyFile);
    fclose(headerFile);
    fclose(bodyFile);

    if (snprintf(cmd, sizeof(cmd),
                 "curl -sS -m 20 -H @%s -H \"Content-Type: application/json\" "
                 "--data-binary @%s \"https://graph.facebook.com/%s/%s/messages\"",
                 headerPath, bodyPath, cfg->graphVersion, cfg->phoneNumberId) >= (int) sizeof(cmd)) {
        unlink(headerPath);
        unlink(bodyPath);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    curl = popen(cmd, "r");
    if (curl == NULL) {
        USER_LOG_ERROR("curl start failed, errno=%d", errno);
        unlink(headerPath);
        unlink(bodyPath);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    {
        size_t n = fread(response, 1, sizeof(response) - 1, curl);
        response[n] = '\0';
    }
    status = pclose(curl);
    unlink(headerPath);
    unlink(bodyPath);

    if (status != 0 || strstr(response, "\"messages\"") == NULL) {
        USER_LOG_ERROR("WhatsApp send failed, status=%d, response=%s", status, response);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    USER_LOG_INFO("WhatsApp group message accepted");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void *Whatsapp_SendTask(void *arg)
{
    T_WhatsappConfig cfg;
    double latDeg = 0.0;
    double lonDeg = 0.0;
    double altM = 0.0;
    double rangeM = 0.0;
    char message[WHATSAPP_TEXT_MAX];
    char window[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN];
    const char *kind = "pointing location";
    T_DjiReturnCode returnCode;
    int wrote;

    (void) arg;
    Whatsapp_Show("C3: reading pointing location");

    returnCode = Whatsapp_LoadConfig(&cfg);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        Whatsapp_Show("C3: set /open_app/whatsapp_pointing.conf");
        goto done;
    }

    returnCode = Whatsapp_ReadLaser(&latDeg, &lonDeg, &altM, &rangeM);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        kind = "aircraft position";
        rangeM = -1.0;
        returnCode = Whatsapp_ReadAircraftGps(&latDeg, &lonDeg, &altM);
    }
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        Whatsapp_Show("C3: no pointing or aircraft position");
        goto done;
    }

    if (rangeM >= 0.0) {
        wrote = snprintf(message, sizeof(message),
                         "Matrice 4T %s\nLat %.7f\nLon %.7f\nAlt %.1f m\nRange %.1f m\n"
                         "https://maps.google.com/?q=%.7f,%.7f",
                         kind, latDeg, lonDeg, altM, rangeM, latDeg, lonDeg);
    } else {
        wrote = snprintf(message, sizeof(message),
                         "Matrice 4T %s\nLat %.7f\nLon %.7f\nAlt %.1f m\n"
                         "https://maps.google.com/?q=%.7f,%.7f",
                         kind, latDeg, lonDeg, altM, latDeg, lonDeg);
    }
    if (wrote < 0 || wrote >= (int) sizeof(message)) {
        Whatsapp_Show("C3: location message too long");
        goto done;
    }

    Whatsapp_Show("C3: sending WhatsApp");
    returnCode = Whatsapp_Post(&cfg, message);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        Whatsapp_Show("C3: WhatsApp send failed");
        goto done;
    }

    snprintf(window, sizeof(window), "C3 sent %.5f, %.5f", latDeg, lonDeg);
    Whatsapp_Show(window);

done:
    pthread_mutex_lock(&s_sendLock);
    s_sendBusy = false;
    pthread_mutex_unlock(&s_sendLock);
    return NULL;
}

T_DjiReturnCode DjiUser_SendPointingLocationToWhatsApp(void)
{
    pthread_t thread;

    pthread_mutex_lock(&s_sendLock);
    if (s_sendBusy) {
        pthread_mutex_unlock(&s_sendLock);
        Whatsapp_Show("C3: WhatsApp send already running");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    s_sendBusy = true;
    pthread_mutex_unlock(&s_sendLock);

    if (pthread_create(&thread, NULL, Whatsapp_SendTask, NULL) != 0) {
        pthread_mutex_lock(&s_sendLock);
        s_sendBusy = false;
        pthread_mutex_unlock(&s_sendLock);
        USER_LOG_ERROR("Create WhatsApp task failed");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    pthread_detach(thread);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

#ifdef WHATSAPP_POINTING_SELFTEST
int main(void)
{
    T_WhatsappConfig cfg;
    char escaped[128];
    FILE *file;

    file = fopen(WHATSAPP_CFG_PATH_CWD, "w");
    if (file == NULL) {
        return 1;
    }
    fputs("access_token=your_access_token\nphone_number_id=123456\ngroup_id=your_group_id\n", file);
    fclose(file);
    if (Whatsapp_LoadConfig(&cfg) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return 2;
    }

    file = fopen(WHATSAPP_CFG_PATH_CWD, "w");
    if (file == NULL) {
        return 3;
    }
    fputs("access_token=EAATestToken\nphone_number_id=123456\ngroup_id=120363@g.us\ngraph_version=v21.0\n", file);
    fclose(file);
    if (Whatsapp_LoadConfig(&cfg) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return 4;
    }
    if (strcmp(cfg.groupId, "120363@g.us") != 0 || strcmp(cfg.phoneNumberId, "123456") != 0) {
        return 5;
    }
    if (Whatsapp_JsonEscape("a\"b\\c\n", escaped, sizeof(escaped)) != 0 ||
        strcmp(escaped, "a\\\"b\\\\c\\n") != 0) {
        fprintf(stderr, "escape mismatch: %s\n", escaped);
        return 6;
    }
    unlink(WHATSAPP_CFG_PATH_CWD);
    printf("selftest ok\n");
    return 0;
}
#endif
