/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/**
 *  Implementation of mpld exported IPC interface
 */
 
 
#include <utils/Log.h>
#include <stdint.h>
#include <sys/types.h>

#include <binder/MemoryHeapBase.h>
#include <Impld.h>

namespace android {

enum {
    GET_SOCKET = IBinder::FIRST_CALL_TRANSACTION,
    ACTIVATE_SENSOR,
    WAKE
};

/* --- Client side --- */
class Bpmpld: public BpInterface<Impld> {
public:
  Bpmpld(const sp<IBinder>& impl) : BpInterface<Impld>(impl)
    {
    }

  virtual int getSocket()
  {
    Parcel data, reply;
    int sock = 0;
    data.writeInterfaceToken(Impld::getInterfaceDescriptor());
    // This will result in a call to the onTransact()
    // method on the server in it's context (from it's binder threads)
    remote()->transact(GET_SOCKET, data, &reply);
    sock = reply.readInt32();
    return sock;
  }
  
  virtual int activateSensor(int client, int sensorType, int activate) {
    Parcel data, reply;
    int rv;
    data.writeInterfaceToken(Impld::getInterfaceDescriptor());
    data.writeInt32(client);
    data.writeInt32(sensorType);
    data.writeInt32(activate);
    remote()->transact(ACTIVATE_SENSOR, data, &reply);
    rv = reply.readInt32();
    return rv;
  }
  virtual int wake(void){
      Parcel data, reply;
       int rv;
       data.writeInterfaceToken(Impld::getInterfaceDescriptor());
       remote()->transact(WAKE, data, &reply);
       rv = reply.readInt32();
       return rv;
  }
};

IMPLEMENT_META_INTERFACE(mpld, "android.vendor.Impld");

/* --- Server side --- */

status_t Bnmpld::onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags) {
  switch (code)
  {
    case GET_SOCKET:
    {
      CHECK_INTERFACE(Impld, data, reply);
      int sock = getSocket();
      reply->writeInt32(sock);
      return NO_ERROR;
      break;
    }
    case ACTIVATE_SENSOR:
    {
        CHECK_INTERFACE(Impld, data, reply);
        int client, sensorType,activate,rv;
        client = data.readInt32();
        sensorType = data.readInt32();
        activate = data.readInt32();
        rv = activateSensor(client, sensorType, activate);
        reply->writeInt32(rv);
        return NO_ERROR;
        break;
    }
    case WAKE:
    {
        CHECK_INTERFACE(Impld, data, reply);
        int rv;
        rv = wake();
        reply->writeInt32(rv);
        return NO_ERROR;
        break;
    }
    default:
      return BBinder::onTransact(code, data, reply, flags);
  }
}

}; // namespace android

