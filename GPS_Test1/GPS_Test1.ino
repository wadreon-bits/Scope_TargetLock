#include <Arduino.h>
#include <Wire.h>

#define ZED_I2C_ADDR  0x42
#define I2C_HZ        100000
#define MAX_UBX       128

#define UBX_NAV   0x01
#define ID_PVT    0x07
#define ID_STATUS 0x03
#define ID_VELNED 0x12

#define READ_INTERVAL_MS 100     // poll every 100 ms (10 Hz)
#define STALE_MS         3000    // no new data for >3s => stale
#define PROBE_PERIOD_MS  1000    // I2C presence check period

// ---------- helpers ----------
static inline uint8_t  U1(const uint8_t *p){ return p[0]; }
static inline uint32_t U4(const uint8_t *p){ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }
static inline int32_t  I4(const uint8_t *p){ return (int32_t)U4(p); }

void ubxChecksum(const uint8_t *data, uint16_t len, uint8_t &ckA, uint8_t &ckB){
  ckA = ckB = 0; for(uint16_t i=0;i<len;i++){ ckA += data[i]; ckB += ckA; }
}

bool gpsPresent(){
  Wire.beginTransmission(ZED_I2C_ADDR);
  return (Wire.endTransmission() == 0);
}

int i2cReadN(uint8_t *dst, size_t n){
  size_t got = 0;
  while(got < n){
    size_t chunk = min((size_t)32, n - got);
    Wire.requestFrom((int)ZED_I2C_ADDR, (int)chunk);
    unsigned long t0 = millis();
    while((int)Wire.available() < (int)chunk && (millis()-t0)<200){}
    while(Wire.available() && got < n) dst[got++] = Wire.read();
    if((millis()-t0)>=200) break;
  }
  return (int)got;
}

int readOneUBX(uint8_t &cls, uint8_t &id, uint16_t &len, uint8_t *payload){
  uint8_t b;
  for(int tries=0; tries<128; ++tries){
    if(i2cReadN(&b,1)!=1) return 0;
    if(b==0xB5){
      if(i2cReadN(&b,1)!=1) return 0;
      if(b==0x62) break;
    }
  }
  uint8_t hdr[4];
  if(i2cReadN(hdr,4)!=4) return 0;
  cls=hdr[0]; id=hdr[1]; len=(uint16_t)(hdr[2]|(hdr[3]<<8));
  if(len>(MAX_UBX-8)) return 0;
  if(i2cReadN(payload,len+2)!=(int)(len+2)) return 0;

  uint8_t ckA,ckB,chk[4+MAX_UBX];
  chk[0]=cls; chk[1]=id; chk[2]=(uint8_t)(len&0xFF); chk[3]=(uint8_t)(len>>8);
  memcpy(chk+4,payload,len);
  ubxChecksum(chk,4+len,ckA,ckB);
  if(ckA!=payload[len]||ckB!=payload[len+1]) return 0;
  return (int)len;
}

// ---------- data state ----------
struct {
  bool connected=false, stale=true;
  uint32_t iTOW=0, lastITOW=0;
  int32_t  lat=0, lon=0;
  uint8_t  fixType=0, numSV=0;
  uint32_t hAcc=0;
  int32_t  gSpeed=0, heading=0;
  unsigned long lastFresh=0, lastProbe=0, lastRead=0;
} S;

// ---------- parsers ----------
void parseNAV_PVT(const uint8_t *p,uint16_t len){
  if(len<92)return;
  S.iTOW=U4(p+0);
  S.lon=I4(p+24);
  S.lat=I4(p+28);
  S.fixType=U1(p+20);
  S.numSV=U1(p+23);
  S.hAcc=U4(p+40);
}

void parseNAV_VELNED(const uint8_t *p,uint16_t len){
  if(len<36)return;
  S.gSpeed=I4(p+20);
  S.heading=I4(p+24);
}

void printSummary(bool stale){
  if(stale){
    Serial.print(F("[STALE] connected=")); Serial.println(S.connected);
    return;
  }
  Serial.print(F("fixType=")); Serial.print(S.fixType);
  Serial.print(F(" SV=")); Serial.print(S.numSV);
  Serial.print(F(" hAcc(m)=")); Serial.print(S.hAcc/1000.0,1);
  Serial.print(F(" lat=")); Serial.print(S.lat*1e-7,7);
  Serial.print(F(" lon=")); Serial.print(S.lon*1e-7,7);
  Serial.print(F(" gSpd(m/s)=")); Serial.print(S.gSpeed/100.0,2);
  Serial.print(F(" head(deg)=")); Serial.println(S.heading*1e-5,2);
}

// ---------- setup / loop ----------
void setup(){
  Serial.begin(115200);
  while(!Serial){}
  Wire.begin();
  Wire.setClock(I2C_HZ);
  Serial.println(F("ZED-F9P efficient UBX reader (100ms poll)"));
}

void loop(){
  unsigned long now = millis();

  // probe presence occasionally
  if(now - S.lastProbe >= PROBE_PERIOD_MS){
    S.lastProbe = now;
    S.connected = gpsPresent();
  }

  // read only every 100 ms
  if (S.connected && (now - S.lastRead) >= READ_INTERVAL_MS) {
  S.lastRead = now;

  uint8_t cls, id;
  uint16_t len;
  static uint8_t payload[MAX_UBX];
  int got = readOneUBX(cls, id, len, payload);

  if (got > 0 && cls == UBX_NAV) {
    if (id == ID_PVT) {
      S.lastITOW = S.iTOW;
      parseNAV_PVT(payload, len);
      if (S.iTOW != S.lastITOW) S.lastFresh = now;
    } else if (id == ID_VELNED) {
      parseNAV_VELNED(payload, len);
    }
  }
  else {
    // ⛔ only flush when read failed or timed out
    int cleared = 0;
    while (cleared < 64) {                 // limit to 64 bytes max
      int n = Wire.requestFrom((int)ZED_I2C_ADDR, 16);
      if (n <= 0) break;
      while (Wire.available()) { (void)Wire.read(); cleared++; }
    }
  }
}

  // mark stale if no fresh data
  S.stale = (!S.connected) || ((now - S.lastFresh) > STALE_MS);

  static unsigned long lastPrint = 0;
  if(now - lastPrint >= 200){ // print 5 Hz
    lastPrint = now;
    printSummary(S.stale);
  }
}
