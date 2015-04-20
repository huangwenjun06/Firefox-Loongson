/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_dom_bluetooth_bluetoothutils_h
#define mozilla_dom_bluetooth_bluetoothutils_h

#include "BluetoothCommon.h"
#include "js/TypeDecls.h"

BEGIN_BLUETOOTH_NAMESPACE

class BluetoothNamedValue;
class BluetoothReplyRunnable;
class BluetoothValue;

//
// BluetoothUuid <-> uuid string conversion
//

/**
 * Convert BluetoothUuid object to xxxxxxxx-xxxx-xxxx-xxxxxxxxxxxx uuid string.
 *
 * Note: This utility function is used by gecko internal only to convert
 * BluetoothUuid created by bluetooth stack to uuid string representation.
 */
void
UuidToString(const BluetoothUuid& aUuid, nsAString& aString);

/**
 * Convert BluetoothUuid object in a reversed byte order to
 * xxxxxxxx-xxxx-xxxx-xxxxxxxxxxxx uuid string.
 * Bluedroid stack reports the BluetoothUuid in a reversed byte order for
 * GATT service, characteristic, descriptor uuids.
 *
 * Note: This utility function is used by gecko internal only to convert
 * BluetoothUuid in a reversed byte order created by bluetooth stack to uuid
 * string representation.
 */
void
ReversedUuidToString(const BluetoothUuid& aUuid, nsAString& aString);

/**
 * Convert xxxxxxxx-xxxx-xxxx-xxxxxxxxxxxx uuid string to BluetoothUuid object.
 *
 * Note: This utility function is used by gecko internal only to convert uuid
 * string created by gecko back to BluetoothUuid representation.
 */
void
StringToUuid(const char* aString, BluetoothUuid& aUuid);

//
// Generate bluetooth signal path from GattId
//

/**
 * Generate bluetooth signal path and UUID string from a GattId.
 *
 * @param aId      [in] GattId value to convert.
 * @param aPath    [out] Bluetooth signal path generated from aId.
 * @param aUuidStr [out] UUID string generated from aId.
 */
void
GeneratePathFromGattId(const BluetoothGattId& aId,
                       nsAString& aPath,
                       nsAString& aUuidStr);

/**
 * Generate bluetooth signal path from a GattId.
 *
 * @param aId   [in] GattId value to convert.
 * @param aPath [out] Bluetooth signal path generated from aId.
 */
void
GeneratePathFromGattId(const BluetoothGattId& aId,
                       nsAString& aPath);

//
// Broadcast system message
//

bool
BroadcastSystemMessage(const nsAString& aType,
                       const BluetoothValue& aData);

bool
BroadcastSystemMessage(const nsAString& aType,
                       const InfallibleTArray<BluetoothNamedValue>& aData);

//
// Dispatch bluetooth reply to main thread
//

#ifdef MOZ_B2G_BT_API_V2
/**
 * Dispatch successful bluetooth reply with NO value to reply request.
 *
 * @param aRunnable  the runnable to reply bluetooth request.
 */
void
DispatchReplySuccess(BluetoothReplyRunnable* aRunnable);

/**
 * Dispatch successful bluetooth reply with value to reply request.
 *
 * @param aRunnable  the runnable to reply bluetooth request.
 * @param aValue     the BluetoothValue to reply successful request.
 */
void
DispatchReplySuccess(BluetoothReplyRunnable* aRunnable,
                     const BluetoothValue& aValue);

/**
 * Dispatch failed bluetooth reply with error string.
 *
 * This function is for methods returning DOMRequest. If |aErrorStr| is not
 * empty, the DOMRequest property 'error.name' would be updated to |aErrorStr|
 * before callback function 'onerror' is fired.
 *
 * NOTE: For methods returning Promise, |aErrorStr| would be ignored and only
 * STATUS_FAIL is returned in BluetoothReplyRunnable.
 *
 * @param aRunnable  the runnable to reply bluetooth request.
 * @param aErrorStr  the error string to reply failed request.
 */
void
DispatchReplyError(BluetoothReplyRunnable* aRunnable,
                   const nsAString& aErrorStr);

/**
 * Dispatch failed bluetooth reply with error status.
 *
 * This function is for methods returning Promise. The Promise would reject
 * with an Exception object that carries nsError associated with |aStatus|.
 * The name and messege of Exception (defined in dom/base/domerr.msg) are
 * filled automatically during promise rejection.
 *
 * @param aRunnable  the runnable to reply bluetooth request.
 * @param aStatus    the error status to reply failed request.
 */
void
DispatchReplyError(BluetoothReplyRunnable* aRunnable,
                   const enum BluetoothStatus aStatus);
#else
// TODO: remove with bluetooth1
void
DispatchBluetoothReply(BluetoothReplyRunnable* aRunnable,
                       const BluetoothValue& aValue,
                       const nsAString& aErrorStr);

// TODO: remove with bluetooth1
void
DispatchStatusChangedEvent(const nsAString& aType,
                           const nsAString& aDeviceAddress,
                           bool aStatus);
#endif

void
DispatchStatusChangedEvent(const nsAString& aType,
                           const nsAString& aDeviceAddress,
                           bool aStatus);

/**
 * Check whether the caller runs at B2G process.
 *
 * @return true if the caller runs at B2G process, false otherwise.
 */
bool
IsMainProcess();

END_BLUETOOTH_NAMESPACE

#endif
