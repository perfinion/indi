
#include <libindi/lilxml.h>
#include <algorithm>
#include <assert.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <string>


namespace
{

static std::map<std::string, std::string> mNicknames;

const char *getDefaultName()
{
    return "AcmeFocuser";
}

// trim from start (in place)
static inline void ltrim(std::string &s)
{
    if (s.empty())
        return;

    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch)
    {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s)
{
    if (s.empty())
        return;

    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch)
    {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static void trim(std::string &s)
{
    ltrim(s);
    rtrim(s);
}

static std::string GetHomeDirectory()
{
    // Check first the HOME environmental variable
    const char *HomeDir = getenv("HOME");

    // ...otherwise get the home directory of the current user.
    // if (!HomeDir)
    // {
    //     HomeDir = getpwuid(getuid())->pw_dir;
    // }
    return (HomeDir ? std::string(HomeDir) : "");
}

/**
 * Nicknames are stored in an xml-format NICKNAME_FILE in a format like the below.
 * Nicknames are assoicated with a driver and stable device identifier.
 *
 * The device identifier must be stable across boots and not dependent on which
 * port the device is plugged into. Usually this will be some form of serial
 * number of the device, but specifics are left up to each driver.
 *
 * Since each identifier is per-driver, devices from different drivers can
 * share the same nickname. The INDI framework must not try to interpret the
 * identifier, only compare for equality.
 *
 * Since the device-name can't be changed once the driver is running, changes
 * to nicknames can only take effect at the next INDI startup.
 *
 * The NicknameTP should be added with addNicknameControl().
 *
 * <INDINicknames>
 *  <nickname driver="AcmeFocuser" identifier="SN123">MainScope</nickname>
 *  <nickname driver="AcmeFocuser" identifier="SN456">GuideScope</nickname>
 *  <nickname driver="AcmeDustCap" identifier="CAP-1-2-3">MainScope</nickname>
 * </INDINicknames>
 */

#define NICKNAME_FILE "/.indi/Nicknames.xml"
#define NICK_TAG_ROOT "INDINicknames"
#define NICK_TAG_DEVICE "device"
#define NICK_ATTR_NAME "name"
#define NICK_TAG_ENTRY "nickname"
#define NICK_ATTR_ID "identifier"

std::string stringXMLEle(XMLEle *ep)
{
    char *buf = (char*)malloc(sprlXMLEle(ep, 0) + 1);
    sprXMLEle(buf, ep, 0);
    std::string ret{buf};
    free(buf);

    return ret;
}

const char *LoadINDINicknamesXML()
{
    const std::string filename = GetHomeDirectory() + NICKNAME_FILE;
    std::map<std::string, std::string> nickmap;

    LilXML *lp = newLilXML();
    XMLEle *NickXmlRoot = nullptr;
    static char errmsg[512];
    memset(errmsg, 0, sizeof(errmsg)); // Clear each run
    FILE *fp = fopen(filename.c_str(), "r");
    if (fp)
    {
        std::cout << "Reading file\n";
        NickXmlRoot = readXMLFile(fp, lp, errmsg);
        fclose(fp);
    }
    delLilXML(lp);

    // NickXmlRoot is the root root tag, it's name is INDINickname
    // iterating that one gives the other tags
    if (!NickXmlRoot || errmsg[0]) {
        std::cout << "Invalid NickXmlRoot: " << NickXmlRoot << std::endl;
        std::cout << "errmsg: " << errmsg << std::endl;
        return errmsg;
    }

    std::cout << "INDINicknames tag: " << tagXMLEle(NickXmlRoot) << std::endl;
    std::cout << "INDINicknames tag:===========\n" << stringXMLEle(NickXmlRoot) << "\n=============\n";

    if (strcmp(tagXMLEle(NickXmlRoot), NICK_TAG_ROOT) != 0) {
        delXMLEle(NickXmlRoot);
        std::cout << "Not an INDI Nickname file\n";
        return 0;
    }

    // children of top level INDINicknames
    // This should be <device>
    XMLEle *devicexml = nextXMLEle(NickXmlRoot, 1);

    if (!devicexml) {
        std::cout << "no children under INDINicknames\n";
        return 0;
    }

    // std::cout << "Nickname tag: " << stringXMLEle(devicexml);
    std::cout << "device tag: " << tagXMLEle(devicexml) << std::endl;

    if (!strcmp(tagXMLEle(devicexml), NICK_TAG_ROOT)) {
        delXMLEle(devicexml);
        std::cout << "Not an INDI Nickname file\n";
        return 0;
    }

    std::cout << "Start Iterating <device>\n";
    XMLEle *nickxml = nullptr;

    for (; devicexml != NULL; devicexml = nextXMLEle(NickXmlRoot, 0))
    {
        // Skip non <device> tags
        if (strcmp(tagXMLEle(devicexml), NICK_TAG_DEVICE))
        {
            printf("Skipping XML Non-Device: %s\n", tagXMLEle(devicexml));
            continue;
        }

        // find name= attr
        const char *devname = findXMLAttValu(devicexml, NICK_ATTR_NAME);
        // skip other drivers
        if (!devname || strcmp(devname, getDefaultName()))
        {
            printf("Skipping XML driver: %s\n", devname);
            continue;
        }

        // Found <device name= matching the current driver
        nickxml = nextXMLEle(devicexml, 1);
        break;
    }

    for (; nickxml != NULL; nickxml = nextXMLEle(devicexml, 0))
    {
        // Skip non <nickname> tags
        if (strcmp(tagXMLEle(nickxml), NICK_TAG_ENTRY))
        {
            printf("Skipping XML Non-Nickname: %s\n", tagXMLEle(devicexml));
            continue;
        }

        // find identifier= attr
        const char *pId = findXMLAttValu(nickxml, NICK_ATTR_ID);
        const char *pVal = pcdataXMLEle(nickxml);
        if (pId && pVal)
        {
            std::string sId{pId}, sVal{pVal}; // deep copy
            trim(sId);
            trim(sVal);
            if (!sId.empty() && !sVal.empty())
            {
                nickmap[sId] = sVal;
                std::cout << " * ID: " << sId << " Val: " << sVal << std::endl;
            }
        }
    }
    std::cout << "Finished Iterating" << std::endl;

    delXMLEle(nickxml);
    delXMLEle(devicexml);
    delXMLEle(NickXmlRoot);

    mNicknames = nickmap;

    return 0;
}

/**
 * @brief Save Nickname data to XML.
 *
 * This needs to re-read the XML file in case other devices wrote data since it was last read.
 * First read the whole file, find entry for this driver (if exists), create if not.
 * Clear old and add new entries for nicknames from mNickname map.
 */
int SaveINDINicknamesXML()
{
    const std::string filename = GetHomeDirectory() + NICKNAME_FILE;

    LilXML *lp = newLilXML();
    XMLEle *NickXmlRoot = nullptr;
    XMLEle *devicexml = nullptr;
    XMLEle *nickxml = nullptr;
    static char errmsg[512];
    memset(errmsg, 0, sizeof(errmsg)); // Clear each run
    FILE *fp = fopen(filename.c_str(), "r+");
    if (fp)
    {
        NickXmlRoot = readXMLFile(fp, lp, errmsg);
    }

    // NickXmlRoot is the root <INDINickname> tag
    if (!NickXmlRoot) // empty file, make new root node
        NickXmlRoot = addXMLEle(nullptr, NICK_TAG_ROOT);

    if (strcmp(tagXMLEle(NickXmlRoot), NICK_TAG_ROOT) != 0) {
        // Unexpected tag, clear and make new
        delXMLEle(NickXmlRoot);
        NickXmlRoot = addXMLEle(nullptr, NICK_TAG_ROOT);
    }

    // children of top level INDINicknames
    // This should be <device>
    devicexml = nextXMLEle(NickXmlRoot, 1);
    for (; devicexml != NULL; devicexml = nextXMLEle(NickXmlRoot, 0))
    {
        // Skip non <device> tags
        if (strcmp(tagXMLEle(devicexml), NICK_TAG_DEVICE))
        {
            printf("Skipping XML Non-Device: %s\n", tagXMLEle(devicexml));
            continue;
        }

        // find name= attr
        const char *devname = findXMLAttValu(devicexml, NICK_ATTR_NAME);
        // skip other drivers
        if (devname && strcmp(devname, getDefaultName()) == 0)
        {
            // Found <device name= matching the current driver
            nickxml = nextXMLEle(devicexml, 1);
            break;
        }
    }

    if (!devicexml)
    {
        // No entry found for current driver, add empty
        devicexml = addXMLEle(NickXmlRoot, NICK_TAG_DEVICE);
        addXMLAtt(devicexml, NICK_ATTR_NAME, getDefaultName());
    }

    // Remove all nicknames in current driver section
    while ((nickxml = nextXMLEle(devicexml, 1)) != 0)
    {
        delXMLEle(nickxml);
        nickxml = nullptr;
    }

    std::cout << "Done removing, count = " << nXMLEle(NickXmlRoot) << "\n";
    std::cout << "INDINicknames after removing :===========\n" << stringXMLEle(NickXmlRoot) << "\n=============\n";
    for (const auto &kv : mNicknames)
    {
        nickxml = addXMLEle(devicexml, NICK_TAG_ENTRY);
        addXMLAtt(nickxml, NICK_ATTR_ID, kv.first.c_str());
        editXMLEle(nickxml, kv.second.c_str());
    }

    // Reopen for writng and truncate file
    fp = freopen(filename.c_str(), "w", fp);
    if (fp)
    {
        prXMLEle(fp, NickXmlRoot, 0);
        fclose(fp);
    }

    delXMLEle(NickXmlRoot);
    delLilXML(lp);
    return 0;
}

} // namespace

int main(int argc, char *argv[])
{
    std::cout << "Starting" << std::endl;

    std::cout << std::endl;
    const char *errmsg = LoadINDINicknamesXML();
    if (errmsg)
        std::cout << "errmsg[0] ret: " << errmsg << std::endl;
    else
        std::cout << "errmsg NULL\n";

    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "Done Loading" << std::endl;
    for (const auto &kv : mNicknames)
    {
        std::cout << " - ID: " << kv.first << " Nick: " << kv.second << std::endl;
    }
    mNicknames["Hello"] = "World";

    SaveINDINicknamesXML();

    return 0;
}
