#ifndef MQTT_HELPER_H
#define MQTT_HELPER_H

bool mqttPublish(const char *topic, const char *message);
bool mqttDebug(const char *message);

#endif