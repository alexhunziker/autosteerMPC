#include <iostream>
#include <string>
#include <memory>
#include <Lidar/LIDARDevice.h>
#include <config.h>
#include <simpleini/SimpleIni.h>

using namespace std;
using namespace ydlidar;

#define PI 3.142

bool stop = false;

bool debug = true;
double distances[160];
double last_valid[160];

// extern C allows to use C-Linkage for next scope, required for ctypes,
extern "C"
{

    int getIndexForAngle(double angleRad)
    {
        return int((angleRad + PI / 2) * 50);
    }

    void disableDebug()
    {
        debug = false;
    }

    double getDistanceForAngle(double angleRad)
    {
        int index = getIndexForAngle(angleRad);
        if (debug)
        {
            cout << "DEBUG: Lidar distance " << distances[index] * 100 << " for angle " << angleRad << "\n";
        }
        return distances[index] * 100;
    }

    double getLastValidForAngle(double angleRad)
    {
        // Returns time in s since UTC epoch
        int index = getIndexForAngle(angleRad);
        if (debug)
        {
            cout << "DEBUG: Lidar timestamp " << (int)last_valid[index] << " for angle " << angleRad << "\n";
        }
        return last_valid[index];
    }

    void adjustedAngleAndDistance(double *angle, double *distance)
    {
        *angle = 23.0;
        *distance = 2.3;
    }
}

std::vector<float> split(const std::string &s, char delim)
{
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while (std::getline(ss, number, delim))
    {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}

void LaserScanCallback(const LaserScan &scan)
{
    if (debug)
    {
        std::cout << "DEBUG: received scan size: " << scan.ranges.size() << std::endl;
        std::cout << "DEBUG: scan   system time: " << scan.system_time_stamp << std::endl;
        std::cout << "DEBUG: scan     frequency: " << 1000000000.0 / scan.config.scan_time << "HZ" << std::endl;
        std::cout << "DEBUG: distance front: " << distances[78] << " measured at " << last_valid[78] << "\n";
    }

    // Loop over received data points
    int stored = 0;
    for (size_t i = 0; i < scan.ranges.size(); i++)
    {
        double angle = scan.config.min_angle + i * scan.config.ang_increment;
        double distance = scan.ranges[i];
        int intensity = scan.intensities[i];

        if (intensity >= 0)
        {
            // Scanning range is also restricted in lidar.ini
            // (only points ahead of vehicule are relevant)
            if (angle < (PI / 2) && angle > (-PI / 2) && distance > 0)
            {
                int idx = getIndexForAngle(angle);
                distances[idx] = distance;
                last_valid[idx] = (double)scan.system_time_stamp / 1000000000.0;
                stored++;
            }
        }
    }
    if (debug)
    {
        std::cout << "Stored data points: " << stored << "\n";
    }
}

extern "C"
{
    void stop_measuring()
    {
        stop = true;
    }

    int main(int argc, char *argv[])
    {
        // Set up variables to read Configuration
        ydlidar::init(argc, argv);
        CSimpleIniA ini;
        ini.SetUnicode();
        LaserParamCfg cfg;
        std::string ini_file = "lidar.ini";

        // Read Configurations
        SI_Error rc = ini.LoadFile(ini_file.c_str());
        if (rc >= 0)
        {
            const char *pszValue = ini.GetValue("LIDAR", "serialPort", "");
            cfg.serialPort = pszValue;

            pszValue = ini.GetValue("LIDAR", "ignoreArray", "");

            cfg.ignoreArray = split(pszValue, ',');

            cfg.serialBaudrate = ini.GetLongValue("LIDAR", "serialBaudrate", cfg.serialBaudrate);
            cfg.sampleRate = ini.GetLongValue("LIDAR", "sampleRate", cfg.sampleRate);
            cfg.scanFrequency = ini.GetLongValue("LIDAR", "scanFrequency", cfg.scanFrequency);

            cfg.intensity = ini.GetBoolValue("LIDAR", "intensity", cfg.intensity);
            cfg.autoReconnect = ini.GetBoolValue("LIDAR", "autoReconnect", cfg.autoReconnect);
            cfg.exposure = ini.GetBoolValue("LIDAR", "exposure", cfg.exposure);
            cfg.fixedResolution = ini.GetBoolValue("LIDAR", "fixedResolution", cfg.fixedResolution);
            cfg.reversion = ini.GetBoolValue("LIDAR", "reversion", cfg.reversion);
            cfg.heartBeat = ini.GetBoolValue("LIDAR", "heartBeat", cfg.heartBeat);

            cfg.maxAngle = ini.GetDoubleValue("LIDAR", "maxAngle", cfg.maxAngle);
            cfg.minAngle = ini.GetDoubleValue("LIDAR", "minAngle", cfg.minAngle);
            cfg.maxRange = ini.GetDoubleValue("LIDAR", "maxRange", cfg.maxRange);
            cfg.minRange = ini.GetDoubleValue("LIDAR", "minRange", cfg.minRange);
        }

        try
        {
            // Configure ydlidar Port and try to auto-fix if possible
            LIDAR ydlidar;
            std::vector<string> ports = ydlidar.getLidarList();

            if (ports.size() > 1)
            {
                printf("ERROR: Multiple possible LIDARs detected, shold be 1");
            }

            if (cfg.serialPort != ports[0])
            {
                printf("WARN: serial port adjusted automatically, as ini was invalid");
                cfg.serialPort = ports[0];
            }

            if (cfg.serialPort.empty())
            {
                printf("ERROR: Serial Port was not set and could not be detected automatically");
            }

            // Start Lidar with parameters
            ydlidar.RegisterLIDARDataCallback(&LaserScanCallback);
            ydlidar.UpdateLidarParamCfg(cfg);

            // Measuring Loop
            while (ydlidar::ok() && !stop)
            {
                try
                {
                    ydlidar.spinOnce();
                }
                catch (TimeoutException &e)
                {
                    std::cout << e.what() << std::endl;
                }
                catch (CorruptedDataException &e)
                {
                    std::cout << e.what() << std::endl;
                }
                catch (DeviceInformationException &e)
                {
                    std::cout << e.what() << std::endl;
                }
                catch (DeviceException &e)
                {
                    std::cerr << e.what() << std::endl;
                    break;
                }
            }
        }
        catch (TimeoutException &e)
        {
            std::cout << e.what() << std::endl;
        }
        catch (CorruptedDataException &e)
        {
            std::cout << e.what() << std::endl;
        }
        catch (DeviceInformationException &e)
        {
            std::cout << e.what() << std::endl;
        }
        catch (DeviceException &e)
        {
            std::cerr << e.what() << std::endl;
        }
        return 0;
    }
}
