#include <EarlyPersist.h>
#include <hidl/HidlTransportSupport.h>
#include <utils/StrongPointer.h>

#ifndef _cplusplus
#define _cplusplus
#include "EarlyPersistHalHelper.h"
#endif

using vendor::harman::earlypersist::V1_0::IEarlyPersist;
using vendor::harman::earlypersist::V1_0::ProductConfig;
using vendor::harman::earlypersist::V1_0::ProductField;
using vendor::harman::earlypersist::V1_0::ProductFieldValue;
using vendor::harman::earlypersist::V1_0::VehicleDataValue;
using ::android::hardware::hidl_vec;
using ::android::hardware::hidl_string;
using namespace std;
using vendor::harman::earlypersist::V1_0::StatusCode;

void readEarlyPersistHalHelper(char *read_buffer)
{
    VehicleDataValue dataVal;
    StatusCode status = StatusCode::NOK;
    hidl_string hidl_key_name("AUDIO_SYSTEM_CONF");
    android::sp <IEarlyPersist> mEarlyPersist;

    mEarlyPersist = IEarlyPersist::getService();
    if (mEarlyPersist.get() == 0) {
        ALOGE("pulseaudio initializeEarlyPersistHal get service failed!");
        return;
    }
    ALOGD("pulseaudio readEarlyPersistHalHelper key_name :: %s ", hidl_key_name.c_str());

    mEarlyPersist->readEarlyPersistData(hidl_key_name,
            [&dataVal, &status]
            (StatusCode s, VehicleDataValue v) {
                ALOGD("return value test");
                status = s;
                if (s == StatusCode::OK) {
                    dataVal = v;
                    ALOGD("pulseaudio readEarlyPersistHalHelper, return status: %d & value : %s",s, v.value.stringValue.c_str());
                }
            });

    strcpy(read_buffer, dataVal.value.stringValue.c_str());
    ALOGD("pulseaudio readEarlyPersistHalHelper, return buffer : %s", read_buffer);
}
