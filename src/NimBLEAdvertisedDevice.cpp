/*
 * NimBLEAdvertisedDevice.cpp
 *
 *  Created: on Jan 24 2020
 *      Author H2zero
 *
 * Originally:
 *
 *  BLEAdvertisedDevice.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: kolban
 */
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)

#include "nimconfig.h"
#if defined(CONFIG_BT_NIMBLE_ROLE_OBSERVER)

#include "NimBLEDevice.h"
#include "NimBLEAdvertisedDevice.h"
#include "NimBLEUtils.h"
#include "NimBLELog.h"

static const char* LOG_TAG = "NimBLEAdvertisedDevice";


/**
 * @brief Constructor
 */
NimBLEAdvertisedDevice::NimBLEAdvertisedDevice() : m_payload(62,0){
    m_advType          = 0;
    m_appearance       = 0;
    m_manufacturerData = "";
    m_name             = "";
    m_rssi             = -9999;
    m_txPower          = 0;
    //m_payloadLength    = 0;

    m_haveAppearance       = false;
    m_haveManufacturerData = false;
    m_haveName             = false;
    m_haveRSSI             = false;
    m_haveServiceData      = false;
    m_haveServiceUUID      = false;
    m_haveTXPower          = false;
    m_callbackSent         = false;

} // NimBLEAdvertisedDevice


/**
 * @brief Get the address of the advertising device.
 * @return The address of the advertised device.
 */
NimBLEAddress NimBLEAdvertisedDevice::getAddress() {
    return m_address;
} // getAddress


/**
 * @brief Get the advertisement type.
 * @return The advertising type the device is reporting:
 * * BLE_HCI_ADV_TYPE_ADV_IND            (0) - indirect advertising
 * * BLE_HCI_ADV_TYPE_ADV_DIRECT_IND_HD  (1) - direct advertisng - high duty cycle
 * * BLE_HCI_ADV_TYPE_ADV_SCAN_IND       (2) - indirect scan response
 * * BLE_HCI_ADV_TYPE_ADV_NONCONN_IND    (3) - indirect advertisng - not connectable
 * * BLE_HCI_ADV_TYPE_ADV_DIRECT_IND_LD  (4) - direct advertising - low duty cycle
 */
uint8_t NimBLEAdvertisedDevice::getAdvType() {
    return m_advType;
} // getAdvType


/**
 * @brief Get the appearance.
 *
 * A %BLE device can declare its own appearance.  The appearance is how it would like to be shown to an end user
 * typcially in the form of an icon.
 *
 * @return The appearance of the advertised device.
 */
uint16_t NimBLEAdvertisedDevice::getAppearance() {
    return m_appearance;
} // getAppearance


/**
 * @brief Get the manufacturer data.
 * @return The manufacturer data of the advertised device.
 */
std::string NimBLEAdvertisedDevice::getManufacturerData() {
    uint8_t data_loc = 0;
    if(findAdvField(BLE_HS_ADV_TYPE_MFG_DATA, 0, &data_loc) > 0) {
        ble_hs_adv_field *field = (ble_hs_adv_field *)&m_payload[data_loc];
        if(field->length > 1) {
            return std::string((char*)field->value, field->length-1);
        }
    }

    return "";
} // getManufacturerData


/**
 * @brief Get the advertised name.
 * @return The name of the advertised device.
 */
std::string NimBLEAdvertisedDevice::getName() {
    uint8_t data_loc = 0;
    if(findAdvField(BLE_HS_ADV_TYPE_COMP_NAME, 0, &data_loc) > 0 ||
       findAdvField(BLE_HS_ADV_TYPE_INCOMP_NAME, 0, &data_loc) > 0)
    {
        ble_hs_adv_field *field = (ble_hs_adv_field *)&m_payload[data_loc];
        if(field->length > 1) {
            return std::string((char*)field->value, field->length-1);
        }
    }

    return "";
} // getName


/**
 * @brief Get the RSSI.
 * @return The RSSI of the advertised device.
 */
int NimBLEAdvertisedDevice::getRSSI() {
    return m_rssi;
} // getRSSI


/**
 * @brief Get the scan object that created this advertised device.
 * @return The scan object.
 */
NimBLEScan* NimBLEAdvertisedDevice::getScan() {
    return NimBLEDevice::getScan();
} // getScan


/**
 * @brief Get the service data.
 * @param [in] index The vector index of the service data requested.
 * @return The advertised service data or empty string if no data.
 */
std::string NimBLEAdvertisedDevice::getServiceData(uint8_t index) {
    ble_hs_adv_field *field = nullptr;
    uint8_t bytes;
    uint8_t data_loc = findServiceData(index, &bytes);

    if(data_loc != 0xFF) {
        field = (ble_hs_adv_field *)&m_payload[data_loc];
        if(field->length >= bytes) {
            return std::string((char*)(field->value + bytes), field->length - bytes);
        }
    }

    return "";
} //getServiceData


/**
 * @brief Get the service data.
 * @param [in] uuid The uuid of the service data requested.
 * @return The advertised service data or empty string if no data.
 */
std::string NimBLEAdvertisedDevice::getServiceData(const NimBLEUUID &uuid) {
    uint8_t serviceDataCount = getServiceDataCount();
    for(uint8_t i = 0; i < serviceDataCount; i++) {
        if(getServiceDataUUID(i) == uuid) {
            return getServiceData(i);
        }
    }
    NIMBLE_LOGW(LOG_TAG, "getServiceData: uuid not found");
    return "";
} //getServiceData


/**
 * @brief Get the UUID of the serice data at the index.
 * @param [in] index The index of the service data UUID requested.
 * @return The advertised service data UUID or an empty UUID if not found.
 */
NimBLEUUID NimBLEAdvertisedDevice::getServiceDataUUID(uint8_t index) {
    ble_hs_adv_field *field = nullptr;
    uint8_t bytes;
    uint8_t data_loc = findServiceData(index, &bytes);

    if(data_loc != 0xFF) {
        field = (ble_hs_adv_field *)&m_payload[data_loc];
        if(field->length >= bytes) {
            return NimBLEUUID(field->value, bytes, false);
        }
    }

    return NimBLEUUID("");
} // getServiceDataUUID


uint8_t NimBLEAdvertisedDevice::findServiceData(uint8_t index, uint8_t* bytes) {
    uint8_t data_loc = 0;
    uint8_t found = 0;

    *bytes = 0;
    found = findAdvField(BLE_HS_ADV_TYPE_SVC_DATA_UUID16, index, &data_loc);
    if(found > 0) {
        if(found == index) {
            *bytes = 2;
        } else {
            index -= found;
        }
    }

    if(!bytes) {
        found = findAdvField(BLE_HS_ADV_TYPE_SVC_DATA_UUID32, index, &data_loc);
        if(found > 0) {
            if(found == index) {
                *bytes = 4;
            } else {
                index -= found;
            }
        }
    }

    if(!bytes) {
        found = findAdvField(BLE_HS_ADV_TYPE_SVC_DATA_UUID128, index, &data_loc);
        if(found > 0) {
            if(found == index) {
                *bytes = 16;
            } else {
                index -= found;
            }
        }
    }

    if(*bytes > 0) {
        return data_loc;
    }

    return uint8_t(0xFF);
}

/**
 * @brief Get the count of advertised service data UUIDS
 * @return The number of service data UUIDS in the vector.
 */
size_t NimBLEAdvertisedDevice::getServiceDataCount() {
    uint8_t count = 0;
    count += findAdvField(BLE_HS_ADV_TYPE_SVC_DATA_UUID16);
    count += findAdvField(BLE_HS_ADV_TYPE_SVC_DATA_UUID32);
    count += findAdvField(BLE_HS_ADV_TYPE_SVC_DATA_UUID128);
    NIMBLE_LOGE(LOG_TAG, "Service data count: %d", m_serviceDataVec.size());
    return count;
} // getServiceDataCount


/**
 * @brief Get the Service UUID.
 * @param [in] index The vector index of the service UUID requested.
 * @return The Service UUID of the advertised service, or an empty UUID if not found.
 */
NimBLEUUID NimBLEAdvertisedDevice::getServiceUUID(uint8_t index) {
    uint8_t data_loc = 0;
    ble_hs_adv_field *field = nullptr;

    if(findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS16, 0, &data_loc) > 0 ||
       findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS16, 0, &data_loc) > 0)
    {
        field = (ble_hs_adv_field *)&m_payload[data_loc];
        if(field->length >= 2 * index) {
            return NimBLEUUID(field->value + 2 * index, 2, false);
        }
        index -= field->length / 2;
    }

    if(findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS32, 0, &data_loc) > 0 ||
       findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS32, 0, &data_loc) > 0)
    {
        field = (ble_hs_adv_field *)&m_payload[data_loc];
        if(field->length >= 4 * index) {
            return NimBLEUUID(field->value + 4 * index, 4, false);
        }
        index -= field->length / 4;
    }

    if(findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS128, 0, &data_loc) > 0 ||
       findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS128, 0, &data_loc) > 0)
    {
        field = (ble_hs_adv_field *)&m_payload[data_loc];
        if(field->length >= 16 * index) {
            return NimBLEUUID(field->value + 16 * index, 16, false);
        }
    }

    return NimBLEUUID("");
} // getServiceUUID


/**
 * @brief Get the number of services advertised
 * @return The count of services in the advertising packet.
 */
size_t NimBLEAdvertisedDevice::getServiceUUIDCount() {
    uint8_t count = 0;
    count += findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS16);
    count += findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS16);
    count += findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS32);
    count += findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS32);
    count += findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS128);
    count += findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS128);
        NIMBLE_LOGE(LOG_TAG, "Service UUID count: %d", m_serviceUUIDs.size());
    return count;
} // getServiceUUIDCount


/**
 * @brief Check advertised services for existance of the required UUID
 * @return Return true if service is advertised
 */
bool NimBLEAdvertisedDevice::isAdvertisingService(const NimBLEUUID &uuid) {
    uint8_t data_loc = 0;
    bool hasSvcs = false;
    ble_hs_adv_field *field = nullptr;
    uint8_t uuidSize = uuid.bitSize() / 8;

    switch(uuidSize * 8) {
        case BLE_UUID_TYPE_16:
            if(findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS16, 0, &data_loc) > 0 ||
               findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS16, 0, &data_loc) > 0)
            {
                hasSvcs = true;
            }
            break;

        case BLE_UUID_TYPE_32:
            if(findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS32, 0, &data_loc) > 0 ||
               findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS32, 0, &data_loc) > 0)
            {
                hasSvcs = true;
            }
            break;

        case BLE_UUID_TYPE_128:
            if(findAdvField(BLE_HS_ADV_TYPE_INCOMP_UUIDS128, 0, &data_loc) > 0 ||
               findAdvField(BLE_HS_ADV_TYPE_COMP_UUIDS128, 0, &data_loc) > 0)
            {
                hasSvcs = true;
            }
            break;

        default:
            break;
    }

    if(hasSvcs) {
        field = (ble_hs_adv_field *)&m_payload[data_loc];
        for(uint8_t i = 0; i < field->length; i += uuidSize) {
            if(uuid == NimBLEUUID((field->value + i), uuidSize, false)) {
                return true;
            }
        }
    }

    return false;
} // isAdvertisingService


/**
 * @brief Get the TX Power.
 * @return The TX Power of the advertised device.
 */
int8_t NimBLEAdvertisedDevice::getTXPower() {
    uint8_t data_loc = 0;
    if(findAdvField(BLE_HS_ADV_TYPE_TX_PWR_LVL, 0, &data_loc) > 0) {
        ble_hs_adv_field *field = (ble_hs_adv_field *)&m_payload[data_loc];
        if(field->length == BLE_HS_ADV_TX_PWR_LVL_LEN+1) {
            return *(int8_t*)field->value;
        }
    }

    return -99;
} // getTXPower


/**
 * @brief Does this advertisement have an appearance value?
 * @return True if there is an appearance value present.
 */
bool NimBLEAdvertisedDevice::haveAppearance() {
    return findAdvField(BLE_HS_ADV_TYPE_APPEARANCE) > 0;
} // haveAppearance


/**
 * @brief Does this advertisement have manufacturer data?
 * @return True if there is manufacturer data present.
 */
bool NimBLEAdvertisedDevice::haveManufacturerData() {
    return findAdvField(BLE_HS_ADV_TYPE_MFG_DATA) > 0;
} // haveManufacturerData


/**
 * @brief Does this advertisement have a name value?
 * @return True if there is a name value present.
 */
bool NimBLEAdvertisedDevice::haveName() {
    return findAdvField(BLE_HS_ADV_TYPE_COMP_NAME) > 0 ||
           findAdvField(BLE_HS_ADV_TYPE_INCOMP_NAME) > 0;
} // haveName


/**
 * @brief Does this advertisement have a signal strength value?
 * @return True if there is a signal strength value present.
 */
bool NimBLEAdvertisedDevice::haveRSSI() {
    return m_rssi != -9999;
} // haveRSSI


/**
 * @brief Does this advertisement have a service data value?
 * @return True if there is a service data value present.
 */
bool NimBLEAdvertisedDevice::haveServiceData() {
    return m_haveServiceData;
} // haveServiceData


/**
 * @brief Does this advertisement have a service UUID value?
 * @return True if there is a service UUID value present.
 */
bool NimBLEAdvertisedDevice::haveServiceUUID() {
    return getServiceUUIDCount() > 0;
} // haveServiceUUID


/**
 * @brief Does this advertisement have a transmission power value?
 * @return True if there is a transmission power value present.
 */
bool NimBLEAdvertisedDevice::haveTXPower() {
    return findAdvField(BLE_HS_ADV_TYPE_TX_PWR_LVL) > 0;
} // haveTXPower


int NimBLEAdvertisedDevice::findAdvField(uint8_t type, uint8_t index, uint8_t *data_loc/*ble_hs_adv_field *out_field*/) {
    ble_hs_adv_field *field = nullptr;
    uint8_t data = 0;
    uint8_t length = m_payload.size();
    int count = 0;

    if(length < 2) {
        NIMBLE_LOGE(LOG_TAG, "Payload length error return");
        return -1;
    }

    while (length > 1) {
        field = (ble_hs_adv_field*)&m_payload[data];

        if (field->length >= length) {
            NIMBLE_LOGE(LOG_TAG, "length error return");
            return -1;
        }

        if (field->type == type) {
            switch(type) {
                case BLE_HS_ADV_TYPE_INCOMP_UUIDS16:
                case BLE_HS_ADV_TYPE_COMP_UUIDS16:
                    count += field->length / 2;
                    break;

                case BLE_HS_ADV_TYPE_INCOMP_UUIDS32:
                case BLE_HS_ADV_TYPE_COMP_UUIDS32:
                    count += field->length / 4;
                    break;

                case BLE_HS_ADV_TYPE_INCOMP_UUIDS128:
                case BLE_HS_ADV_TYPE_COMP_UUIDS128:
                    count += field->length / 16;
                    break;

                default:
                    count++;
                    break;
            }

            NIMBLE_LOGE(LOG_TAG, "Found field %d = %d, length %d value %02x pointer %p", field->type, type, field->length, *field->value, field->value);
            if(data_loc != nullptr) {
                if(index == 0 || count == index) {
                    break;
                }
            }
        }

        length -= 1 + field->length;
        data += 1 + field->length;
    }

  /*  if(index > count) {
        NIMBLE_LOGE(LOG_TAG, "index error return");
        return -1;
    }*/

    if(data_loc != nullptr && field != nullptr) {
        *data_loc = data;
    }

    return count;
}

/**
 * @brief Parse the advertising pay load.
 *
 * The pay load is a buffer of bytes that is either 31 bytes long or terminated by
 * a 0 length value.  Each entry in the buffer has the format:
 * [length][type][data...]
 *
 * The length does not include itself but does include everything after it until the next record.  A record
 * with a length value of 0 indicates a terminator.
 *
 * https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
 */
 void NimBLEAdvertisedDevice::parseAdvertisement() {
    struct ble_hs_adv_fields fields;
    int rc = ble_hs_adv_parse_fields(&fields, &m_payload[0], m_payload.size());
    if (rc != 0) {
        NIMBLE_LOGE(LOG_TAG, "Gap Event Parse ERROR.");
        return;
    }

    if (fields.uuids16 != NULL) {
        for (int i = 0; i < fields.num_uuids16; i++) {
            setServiceUUID(NimBLEUUID(fields.uuids16[i].value));
        }
    }

    if (fields.uuids32 != NULL) {
        for (int i = 0; i < fields.num_uuids32; i++) {
            setServiceUUID(NimBLEUUID(fields.uuids32[i].value));
        }
    }

    if (fields.uuids128 != NULL) {
        for (int i = 0; i < fields.num_uuids128; i++) {
            setServiceUUID(NimBLEUUID(&fields.uuids128[i]));
        }
    }

    if (fields.name != NULL) {
        setName(std::string(reinterpret_cast<char*>(fields.name), fields.name_len));
    }

    if (fields.tx_pwr_lvl_is_present) {
        setTXPower(fields.tx_pwr_lvl);
    }

    if (fields.svc_data_uuid16 != NULL ||
        fields.svc_data_uuid32 != NULL ||
        fields.svc_data_uuid128 != NULL)
    {
        ble_hs_adv_field *field;
        uint8_t *data = &m_payload[0];
        uint8_t length = m_payload.size();
        while(length > 1) {
            field = (ble_hs_adv_field*)data;

            if(field->length > length) {
                break;
            }

            if(field->type == BLE_HS_ADV_TYPE_SVC_DATA_UUID16) {
                if(field->length > 2) {
                    uint16_t uuid;
                    memcpy(&uuid, field->value, 2);
                    setServiceData(NimBLEUUID(uuid), std::string(reinterpret_cast<char*>(field->value + 2), field->length - 3));
                }
            }

            if(field->type == BLE_HS_ADV_TYPE_SVC_DATA_UUID32) {
                if(field->length > 4) {
                    uint32_t uuid;
                    memcpy(&uuid, field->value, 4);
                    setServiceData(NimBLEUUID(uuid), std::string(reinterpret_cast<char*>(field->value + 4), field->length - 5));
                }
            }

            if(field->type == BLE_HS_ADV_TYPE_SVC_DATA_UUID128) {
                if(field->length > 16) {
                    NimBLEUUID uuid(field->value, (size_t)16, false);
                    setServiceData(uuid, std::string(reinterpret_cast<char*>(field->value + 16), field->length - 17));
                }
            }

            length -= 1 + field->length;
            data += 1 + field->length;
        }
    }

    if (fields.appearance_is_present) {
        setAppearance(fields.appearance);
    }

    if (fields.mfg_data != NULL) {
        setManufacturerData(std::string(reinterpret_cast<char*>(fields.mfg_data), fields.mfg_data_len));
    }

/* TODO: create storage and fucntions for these parameters
    if (fields.public_tgt_addr != NULL) {
        NIMBLE_LOGD(LOG_TAG, "    public_tgt_addr=");
        u8p = fields.public_tgt_addr;
        for (i = 0; i < fields.num_public_tgt_addrs; i++) {
            NIMBLE_LOGD(LOG_TAG, "public_tgt_addr=%s ", addr_str(u8p));
            u8p += BLE_HS_ADV_PUBLIC_TGT_ADDR_ENTRY_LEN;
        }
        NIMBLE_LOGD(LOG_TAG, "\n");
    }

    if (fields.slave_itvl_range != NULL) {
        NIMBLE_LOGD(LOG_TAG, "    slave_itvl_range=");
        print_bytes(fields.slave_itvl_range, BLE_HS_ADV_SLAVE_ITVL_RANGE_LEN);
        NIMBLE_LOGD(LOG_TAG, "\n");
    }

    if (fields.adv_itvl_is_present) {
        NIMBLE_LOGD(LOG_TAG, "    adv_itvl=0x%04x\n", fields.adv_itvl);
    }

    if (fields.uri != NULL) {
        NIMBLE_LOGD(LOG_TAG, "    uri=");
        print_bytes(fields.uri, fields.uri_len);
        NIMBLE_LOGD(LOG_TAG, "\n");
    }
*/

 } //parseAdvertisement


/**
 * @brief Set the address of the advertised device.
 * @param [in] address The address of the advertised device.
 */
void NimBLEAdvertisedDevice::setAddress(NimBLEAddress address) {
    m_address = address;
} // setAddress


/**
 * @brief Set the adFlag for this device.
 * @param [in] The discovered adFlag.
 */
void NimBLEAdvertisedDevice::setAdvType(uint8_t advType) {
    m_advType = advType;
} // setAdvType


/**
 * @brief Set the appearance for this device.
 * @param [in] The discovered appearance.
 */
void NimBLEAdvertisedDevice::setAppearance(uint16_t appearance) {
    m_appearance     = appearance;
    m_haveAppearance = true;
} // setAppearance


/**
 * @brief Set the manufacturer data for this device.
 * @param [in] The discovered manufacturer data.
 */
void NimBLEAdvertisedDevice::setManufacturerData(std::string manufacturerData) {
    m_manufacturerData     = manufacturerData;
    m_haveManufacturerData = true;
} // setManufacturerData


/**
 * @brief Set the name for this device.
 * @param [in] name The discovered name.
 */
void NimBLEAdvertisedDevice::setName(std::string name) {
    m_name     = name;
    m_haveName = true;
} // setName


/**
 * @brief Set the RSSI for this device.
 * @param [in] rssi The discovered RSSI.
 */
void NimBLEAdvertisedDevice::setRSSI(int rssi) {
    m_rssi     = rssi;
    m_haveRSSI = true;
} // setRSSI


/**
 * @brief Set the Service UUID for this device.
 * @param [in] serviceUUID The discovered serviceUUID
 */

void NimBLEAdvertisedDevice::setServiceUUID(const char* serviceUUID) {
    return setServiceUUID(NimBLEUUID(serviceUUID));
} // setServiceUUID


/**
 * @brief Set the Service UUID for this device.
 * @param [in] serviceUUID The discovered serviceUUID
 */
void NimBLEAdvertisedDevice::setServiceUUID(NimBLEUUID serviceUUID) {
    // Don't add duplicates
    for (int i = 0; i < m_serviceUUIDs.size(); i++) {
        if (m_serviceUUIDs[i] == serviceUUID) {
            return;
        }
    }
    m_serviceUUIDs.push_back(serviceUUID);
    m_haveServiceUUID = true;
} // setServiceUUID


/**
 * @brief Set the ServiceData value.
 * @param [in] uuid The UUID that the service data belongs to.
 * @param [in] data The service data.
 */
void NimBLEAdvertisedDevice::setServiceData(NimBLEUUID uuid, std::string data) {
    m_haveServiceData = true;
    for(auto &it : m_serviceDataVec) {
        if(it.first == uuid) {
            it.second = data;
            return;
        }
    }
    m_serviceDataVec.push_back({uuid, data});
} //setServiceData


/**
 * @brief Set the power level for this device.
 * @param [in] txPower The discovered power level.
 */
void NimBLEAdvertisedDevice::setTXPower(int8_t txPower) {
    m_txPower     = txPower;
    m_haveTXPower = true;
} // setTXPower


/**
 * @brief Create a string representation of this device.
 * @return A string representation of this device.
 */
std::string NimBLEAdvertisedDevice::toString() {
    std::string res = "Name: " + getName() + ", Address: " + getAddress().toString();

    if (haveAppearance()) {
        char val[6];
        snprintf(val, sizeof(val), "%d", getAppearance());
        res += ", appearance: ";
        res += val;
    }

    if (haveManufacturerData()) {
        char *pHex = NimBLEUtils::buildHexData(nullptr, (uint8_t*)getManufacturerData().data(), getManufacturerData().length());
        res += ", manufacturer data: ";
        res += pHex;
        free(pHex);
    }

    if (haveServiceUUID()) {
        res += ", serviceUUID: " + getServiceUUID().toString();
    }

    if (haveTXPower()) {
        char val[5];
        snprintf(val, sizeof(val), "%d", getTXPower());
        res += ", txPower: ";
        res += val;
    }

    if(haveServiceData()) {
        size_t count = getServiceDataCount();
        res += "\nService Data:";
        for(size_t i = 0; i < count; i++) {
            res += "\nUUID: " + std::string(getServiceDataUUID(i));
            res += ", Data: " + getServiceData(i);
        }
    }

    return res;

} // toString


/**
 * @brief Get the payload advertised by the device.
 * @return The advertisement payload.
 */
uint8_t* NimBLEAdvertisedDevice::getPayload() {
    return &m_payload[0];
} // getPayload


/**
 * @brief Stores the payload of the advertised device in a vector.
 * @param [in] payload The advertisement payload.
 * @param [in] length The length of the payload in bytes.
 * @param [in] append Indicates if the the data should be appended (scan response).
 */
void NimBLEAdvertisedDevice::setPayload(uint8_t *payload, uint8_t length, bool append) {
    if(!append) {
        m_payload.assign(payload, payload + length);
    } else {
        m_payload.insert(m_payload.end(), payload, payload + length);
    }
}


/**
 * @brief Get the advertised device address type.
 * @return The device address type:
 * * BLE_ADDR_PUBLIC      (0x00)
 * * BLE_ADDR_RANDOM      (0x01)
 * * BLE_ADDR_PUBLIC_ID   (0x02)
 * * BLE_ADDR_RANDOM_ID   (0x03)
 */
uint8_t NimBLEAdvertisedDevice::getAddressType() {
    return m_address.getType();
} // getAddressType


/**
 * @brief Get the timeStamp of when the device last advertised.
 * @return The timeStamp of when the device was last seen.
 */
time_t NimBLEAdvertisedDevice::getTimestamp() {
    return m_timestamp;
} // getTimestamp


/**
 * @brief Get the length of the payload advertised by the device.
 * @return The size of the payload in bytes.
 */
size_t NimBLEAdvertisedDevice::getPayloadLength() {
    return m_payload.size();
} // getPayloadLength


#endif // #if defined( CONFIG_BT_NIMBLE_ROLE_CENTRAL)
#endif /* CONFIG_BT_ENABLED */

