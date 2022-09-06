/*************************************/
/* General                           */
/*************************************/
#include <SPI.h>
#include <SD.h>
#include <XBee.h>
#include <Printers.h>
#include <zigbee.h>

#define SS 53
#define Serial_XBee Serial1
#define Serial_Bluetooth Serial2
#define Serial_WiFi Serial3

File f_wifi, f_ble, f_xbee;

/*************************************/
/* BLE                               */
/*************************************/

char c_ble = ' ';

void delayAndRead() {
  delay(50);
  while(Serial_Bluetooth.available())
  {
    c_ble = Serial_Bluetooth.read();
  }
  delay(800);
}

void initHC05ToInq() {
    Serial_Bluetooth.println("AT+CMODE=1");// Enable connect to any device
    delayAndRead();
    Serial_Bluetooth.println("AT+ROLE=1");// Set to master in order to enable scanning
    delayAndRead();
    Serial_Bluetooth.println("AT+INQM=1,10,24");//RSSI, Max 10 devices, ~30s
    delayAndRead();
    Serial_Bluetooth.println("AT+CLASS=0");// Disable COD filter
    delayAndRead();
    Serial_Bluetooth.println("AT+INIT");// Init.
    delayAndRead();
}

/*************************************/
/* XBEE                              */
/*************************************/

/* Code adapted from ZdpScan*/

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error This code relies on little endian integers!
#endif

XBeeWithCallbacks xbee;

/** Helper to generate sequential Zdo transaction identifiers */
uint8_t getNextTransactionId() {
  static uint8_t id = 0;
  return id++;
}

#ifndef lengthof
#define lengthof(x) (sizeof(x)/sizeof(*x))
#endif

/**
 * Helper function to print a field name, followed by the hexadecimal
 * value and a newline.
 */
template <typename T>
static void printField(const __FlashStringHelper *prefix, T data);
template <typename T>
static void printField(const __FlashStringHelper *prefix, T data) {
  f_xbee.print(prefix);
  printHex(f_xbee, data);
  f_xbee.println();
}

void printActiveEndpoints(const zdo_active_ep_rsp_header_t *rsp) {
  f_xbee.println(F("Active endpoints response"));
  printField(F("  About: 0x"), rsp->network_addr_le);
  f_xbee.print(F("  Endpoints found: 0x"));
  printHex(f_xbee, rsp->endpoints, rsp->ep_count, F(", 0x"), NULL);
  f_xbee.println();
}

void printClusters(const __FlashStringHelper *prefix, uint16_t* clusters, uint8_t count) {
  f_xbee.print(prefix);
  for (uint8_t i = 0; i < count; ++i) {
    if (i > 0) f_xbee.print(F(", "));
    f_xbee.print(F("0x"));
    printHex(f_xbee, ((uint16_t*)clusters)[i]);
  }
  if (!count) f_xbee.print(F("none"));

  f_xbee.println();
}

void printSimpleDescriptor(zdo_simple_desc_resp_header_t *rsp) {
  zdo_simple_desc_header_t *desc = (zdo_simple_desc_header_t*)((uint8_t*)rsp + sizeof(zdo_simple_desc_resp_header_t));
  uint8_t *clusters = ((uint8_t*)desc + sizeof(zdo_simple_desc_header_t));

  f_xbee.println(F("Simple descriptor response"));
  printField(F("  About: 0x"), rsp->network_addr_le);
  printField(F("  Endpoint: 0x"), desc->endpoint);
  printField(F("  Profile ID: 0x"), desc->profile_id_le);
  printField(F("  Device ID: 0x"), desc->device_id_le);
  printField(F("  Device Version: "), (uint8_t)(desc->device_version & 0xf));

  uint8_t ip_count = *clusters++;
  printClusters(F("  Input clusters: "), (uint16_t*)clusters, ip_count);
  clusters += 2*ip_count;
  uint8_t op_count = *clusters++;
  printClusters(F("  Output clusters: "), (uint16_t*)clusters, op_count);
}

/* Matching function that can be passed to waitFor() that matches
 * replies to Zdo requests. The data passed along is the Zdo transaction
 * id that was used for the request, which will be used to select the
 * right reply.
 */
bool matchZdoReply(ZBExplicitRxResponse& rx, uintptr_t data) {
  uint8_t *payload = rx.getFrameData() + rx.getDataOffset();
  uint8_t transactionId = (uint8_t)data;

  return rx.getSrcEndpoint() == 0 &&
         rx.getDstEndpoint() == 0 &&
         rx.getProfileId() == WPAN_PROFILE_ZDO &&
         payload[0] == transactionId;
}

/**
 * Create a tx request to send a Zdo request.
 */
ZBExplicitTxRequest buildZdoRequest(XBeeAddress64 addr, uint16_t cluster_id, uint8_t *payload, size_t len) {
  ZBExplicitTxRequest tx(addr, payload, len);
  tx.setSrcEndpoint(WPAN_ENDPOINT_ZDO);
  tx.setDstEndpoint(WPAN_ENDPOINT_ZDO);
  tx.setClusterId(cluster_id);
  tx.setProfileId(WPAN_PROFILE_ZDO);
  tx.setFrameId(xbee.getNextFrameId());
  return tx;
}

/**
 * Create a zdo request, send it and wait for a reply (which will be
 * stored in the given response object).
 * Returns true when a response was received, returns false if something
 * goes wrong (an error message will have been prined already).
 */
bool handleZdoRequest(const __FlashStringHelper *msg, ZBExplicitRxResponse& rx, XBeeAddress64 addr, uint16_t cluster_id, uint8_t *payload, size_t len) {
  ZBExplicitTxRequest tx = buildZdoRequest(addr, cluster_id, (uint8_t*)payload, len);
  xbee.send(tx);

  uint8_t transaction_id = payload[0];
  // This waits up to 5000 seconds, since the default TX timeout (NH
  // value of 1.6s, times three retries) is 4.8s.
  uint8_t status = xbee.waitFor(rx, 5000, matchZdoReply, transaction_id, tx.getFrameId());
  switch(status) {
    case 0: // Success
      return true;
    case XBEE_WAIT_TIMEOUT:
      f_xbee.print(F("No reply received from 0x"));
      printHex(f_xbee, addr.getMsb());
      printHex(f_xbee, addr.getLsb());
      f_xbee.print(F(" while "));
      f_xbee.print(msg);
      f_xbee.println(F("."));
      return false;
    default:
      f_xbee.print(F("Failed to send to 0x"));
      printHex(f_xbee, addr.getMsb());
      printHex(f_xbee, addr.getLsb());
      f_xbee.print(F(" while "));
      f_xbee.print(msg);
      f_xbee.print(F(". Status: 0x"));
      printHex(f_xbee, status);
      f_xbee.println();
      return false;
  }
}

/**
 * Request a list of active endpoints from the node with the given
 * address. Print the endpoints discovered and then request more details
 * for each of the endpoints and print those too.
 */
void get_active_endpoints(XBeeAddress64& addr, uint16_t addr16) {
  zdo_active_ep_req_t payload = {
    .transaction = getNextTransactionId(),
    .network_addr_le = addr16,
  };
  printField(F("Discovering services on 0x"), addr16);

  ZBExplicitRxResponse rx;
  if (!handleZdoRequest(F("requesting active endpoints"),
                        rx, addr, ZDO_ACTIVE_EP_REQ,
                        (uint8_t*)&payload, sizeof(payload)))
    return;

  zdo_active_ep_rsp_header_t *rsp = (zdo_active_ep_rsp_header_t*)(rx.getFrameData() + rx.getDataOffset());

  if (rsp->status) {
    printField(F("Active endpoints request rejected. Status: 0x"), rsp->status);
    return;
  }

  printActiveEndpoints(rsp);

  // Copy the endpoint list, since requesting a descriptor below will
  // invalidate the data in rx / rsp.
  uint8_t endpoints[rsp->ep_count];
  memcpy(endpoints, rsp->endpoints, sizeof(endpoints));

  // Request the simple descriptor for each endpoint
  for (uint8_t i = 0; i < sizeof(endpoints); ++i)
    get_simple_descriptor(addr, addr16, endpoints[i]);
}

void get_simple_descriptor(XBeeAddress64& addr, uint16_t addr16, uint8_t endpoint) {
  zdo_simple_desc_req_t payload = {
    .transaction = getNextTransactionId(),
    .network_addr_le = addr16,
    .endpoint = endpoint,
  };

  ZBExplicitRxResponse rx;
  if (!handleZdoRequest(F("requesting simple descriptor"),
                        rx, addr, ZDO_SIMPLE_DESC_REQ,
                        (uint8_t*)&payload, sizeof(payload)))
    return;

  zdo_simple_desc_resp_header_t *rsp = (zdo_simple_desc_resp_header_t*)(rx.getFrameData() + rx.getDataOffset());

  if (rsp->status) {
    printField(F("Failed to fetch simple descriptor. Status: 0x"), rsp->status);
    return;
  }

  printSimpleDescriptor(rsp);
}

bool getAtValue(uint8_t cmd[2], uint8_t *buf, size_t len, uint16_t timeout = 150) {
  AtCommandRequest req(cmd);
  req.setFrameId(xbee.getNextFrameId());
  uint8_t status = xbee.sendAndWait(req, timeout);
  if (status != 0) {
    f_xbee.print(F("Failed to read "));
    f_xbee.write(cmd, 2);
    f_xbee.print(F(" command. Status: 0x"));
    f_xbee.println(status, HEX);
    return false;
  }

  AtCommandResponse response;
  xbee.getResponse().getAtCommandResponse(response);
  if (response.getValueLength() != len) {
    f_xbee.print(F("Unexpected response length in "));
    f_xbee.write(cmd, 2);
    f_xbee.println(F(" response"));
    return false;
  }

  memcpy(buf, response.getValue(), len);
  return true;
}

// Invert the endianness of a given buffer
void invertEndian(uint8_t *buf, size_t len) {
  for (uint8_t i = 0, j = len - 1; i < len/2; ++i, j--) {
    uint8_t tmp = buf[i];
    buf[i] = buf[j];
    buf[j] = tmp;
  }
}

/**
 * Struct to keep info about discovered nodes.
 */
struct node_info {
  XBeeAddress64 addr64;
  uint16_t addr16;
  uint8_t type: 2;
  uint8_t visited: 1;
};

/**
 * List of nodes found.
 */
node_info nodes[10];
uint8_t nodes_found = 0;

/**
 * Scan the network and discover all other nodes by traversing neighbour
 * tables. The discovered nodes are stored in the nodes array.
 */
void scan_network() {

  // Fetch our operating PAN ID, to filter the LQI results
  uint8_t pan_id[8];
  getAtValue((uint8_t*)"OP", pan_id, sizeof(pan_id));
  // XBee sends in big-endian, but ZDO requests use little endian. For
  // easy comparsion, convert to little endian
  invertEndian(pan_id, sizeof(pan_id));

  // Fetch the addresses of the local node
  XBeeAddress64 local;
  uint8_t shbuf[4], slbuf[4], mybuf[2];
  if (!getAtValue((uint8_t*)"SH", shbuf, sizeof(shbuf)) ||
      !getAtValue((uint8_t*)"SL", slbuf, sizeof(slbuf)) ||
      !getAtValue((uint8_t*)"MY", mybuf, sizeof(mybuf)))
    return;

  nodes[0].addr64.setMsb((uint32_t)shbuf[0] << 24 | (uint32_t)shbuf[1] << 16 | (uint32_t)shbuf[2] << 8 | shbuf[3]);
  nodes[0].addr64.setLsb((uint32_t)slbuf[0] << 24 | (uint32_t)slbuf[1] << 16 | (uint32_t)slbuf[2] << 8 | slbuf[3]);
  nodes[0].addr16 = (uint16_t)mybuf[0] << 8 | mybuf[1];
  nodes[0].type = ZDO_MGMT_LQI_REQ_TYPE_UNKNOWN;
  nodes[0].visited = false;
  nodes_found = 1;

  f_xbee.print(F("0) 0x"));
  printHex(f_xbee, nodes[0].addr64);
  f_xbee.print(F(" (0x"));
  printHex(f_xbee, nodes[0].addr16);
  f_xbee.println(F(", Self)"));

  // nodes[0] now contains our own address, the rest is invalid. We
  // explore the network by asking for LQI info (neighbour table).
  // Initially, this pretends to send a packet to ourselves, which the
  // XBee firmware conveniently handles by pretending that a reply was
  // received (with one caveat: it seems the reply arrives _before_ the
  // TX status).
  uint8_t next = 0;
  do {
    // Query node i for its LQI table
    zdo_mgmt_lqi_req_t payload = {
      .transaction = getNextTransactionId(),
      .start_index = 0,
    };

    do {
      ZBExplicitRxResponse rx;
      if (!handleZdoRequest(F("requesting LQI/neighbour table"),
                            rx, nodes[next].addr64, ZDO_MGMT_LQI_REQ,
                            (uint8_t*)&payload, sizeof(payload)))
        break;

      zdo_mgmt_lqi_rsp_t *rsp = (zdo_mgmt_lqi_rsp_t*)(rx.getFrameData() + rx.getDataOffset());
      if (rsp->status != 0) {
        if (rsp->status != ZDO_STATUS_NOT_SUPPORTED) {
          f_xbee.print(F("LQI query rejected by 0x"));
          printHex(f_xbee, nodes[next].addr16);
          f_xbee.print(F(". Status: 0x"));
          printHex(f_xbee, rsp->status);
          f_xbee.println();
        }
        break;
      }

      if (rsp->start_index != payload.start_index) {
        f_xbee.println(F("Unexpected start_index, skipping this node"));
        break;
      }

      for (uint8_t i = 0; i < rsp->list_count; ++i) {
        zdo_mgmt_lqi_entry_t *e = &rsp->entries[i];
        node_info *n = &nodes[nodes_found];

        if (memcmp(&e->extended_pan_id_le, &pan_id, sizeof(pan_id)) != 0) {
          f_xbee.println(F("Ignoring node in other PAN"));
          continue;
        }

        // Skip if we know about this node already
        uint8_t dup;
        for (dup = 0; dup < nodes_found; ++dup) {
          if (nodes[dup].addr16 == e->nwk_addr_le)
            break;
        }
        if (dup != nodes_found)
          continue;

        n->addr64.setMsb(e->extended_addr_le >> 32);
        n->addr64.setLsb(e->extended_addr_le);
        n->addr16 = e->nwk_addr_le;
        n->type = e->flags0 & 0x3;

        f_xbee.print(nodes_found);
        f_xbee.print(F(") 0x"));
        printHex(f_xbee, n->addr64);
        f_xbee.print(F(" (0x"));
        printHex(f_xbee, n->addr16);
        switch (n->type) {
          case ZDO_MGMT_LQI_REQ_TYPE_COORDINATOR:
            f_xbee.println(F(", Coordinator)"));
            break;
          case ZDO_MGMT_LQI_REQ_TYPE_ROUTER:
            f_xbee.println(F(", Router)"));
            break;
          case ZDO_MGMT_LQI_REQ_TYPE_ENDDEVICE:
            f_xbee.println(F(", End device)"));
            break;
          case ZDO_MGMT_LQI_REQ_TYPE_UNKNOWN:
            f_xbee.println(F(", Unknown)"));
            break;
        }
        nodes_found++;

        if (nodes_found == lengthof(nodes)) {
          f_xbee.println(F("Device table full, terminating network scan"));
          return;
        }
      }

      // Got all neighbours available? Done.
      if (rsp->start_index + rsp->list_count >= rsp->table_entries)
        break;
      // More left? Loop and get more.
      payload.start_index += rsp->list_count;
      payload.transaction = getNextTransactionId();
    } while (true);

    // Done with this node, on to the next
    nodes[next].visited = true;
    ++next;
  } while (next < nodes_found);
}

void setup(void) {
  
  SD.begin(SS);
  f_wifi = SD.open("wifi.txt", FILE_WRITE);
  f_ble = SD.open("ble.txt", FILE_WRITE);
  f_xbee = SD.open("xbee.txt", FILE_WRITE);

  /* Wi-Fi */
  Serial_WiFi.begin(115200);

  /* BLE */

  // HC-05 default serial speed for AT mode is 38400
  Serial_Bluetooth.begin(38400);  

  // Set correct states for inq
  initHC05ToInq();

  // Start inq
  Serial_Bluetooth.println("AT+INQ");

  /* Zigbee */
  Serial_XBee.begin(9600);
  xbee.setSerial(Serial_XBee);

  xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)&Serial_XBee);

  // Set AO=1 to receive explicit RX frames
  // Because this does not write to flash with WR, AO should be reverted
  // on reboot.
  uint8_t value = 1;
  AtCommandRequest req((uint8_t*)"AO", &value, sizeof(value));
  req.setFrameId(xbee.getNextFrameId());
  uint8_t status = xbee.sendAndWait(req, 150);

  // Failed to set AO, aborting scan
  if (status != 0)
    return;

  scan_network();
}


void loop() {

  /* Wi-Fi Scan */
  if(Serial_WiFi.available( ) > 0) {
    f_wifi.print(Serial_WiFi.read());
  }

  /* Bluetooth Scan*/
  
  if (Serial_Bluetooth.available()) {
    char c = Serial_Bluetooth.read();
    f_ble.print(c);
    if(c == '\n'){
      // Restart INQ
      Serial_Bluetooth.println("AT+INQ");
    }
  }
  
  /* Zigbee scan */
  scan_network();
  xbee.loop();

  /* 1min delay between scans */
  delay(60000);
}
