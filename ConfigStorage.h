#ifndef CONFIG_STORAGE_H
#define CONFIG_STORAGE_H

#include "g_head.h"
#include <Preferences.h>

class ConfigStorage {
public:
    ConfigStorage();
    void begin();
    void loadConfig(MyConfig &config);
    void saveConfig(const MyConfig &config);
    void resetConfig(MyConfig &config);

private:
    Preferences preferences;
};

#endif
