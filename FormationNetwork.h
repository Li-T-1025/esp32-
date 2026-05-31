#ifndef FORMATION_NETWORK_H
#define FORMATION_NETWORK_H

#include "g_head.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

class FormationNetwork {
public:
    FormationNetwork(MyConfig &config, MyStatus &status, MyLine *lines);
    void begin();
    void handle();
    bool isConnected();

private:
    MyConfig &_config;
    MyStatus &_status;
    MyLine *_lines;
    WebServer server;
    DNSServer dnsServer;
    unsigned long lastStaReconnectAttempt;
    unsigned long lastStaConnectStart;
    bool lastStaConnected;
    wl_status_t lastStaWifiStatus;

    void setupAP();
    void setupSTA();
    void maintainSTAConnection();
    
    void handleRoot();
    void handleSave();
    void handleStatus();
    void handleConfig();
    
    String getHTML();
};

#endif
