/* -*- Mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; tab-width: 40 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_dom_bluetooth_bluetoothgattdescriptor_h__
#define mozilla_dom_bluetooth_bluetoothgattdescriptor_h__

#include "mozilla/Attributes.h"
#include "mozilla/dom/BluetoothGattDescriptorBinding.h"
#include "mozilla/dom/bluetooth/BluetoothCommon.h"
#include "nsCOMPtr.h"
#include "nsWrapperCache.h"
#include "nsPIDOMWindow.h"

BEGIN_BLUETOOTH_NAMESPACE

class BluetoothGattCharacteristic;
class BluetoothSignal;
class BluetoothValue;

class BluetoothGattDescriptor final : public nsISupports
                                    , public nsWrapperCache
{
public:
  NS_DECL_CYCLE_COLLECTING_ISUPPORTS
  NS_DECL_CYCLE_COLLECTION_SCRIPT_HOLDER_CLASS(BluetoothGattDescriptor)

  /****************************************************************************
   * Attribute Getters
   ***************************************************************************/
  BluetoothGattCharacteristic* Characteristic() const
  {
    return mCharacteristic;
  }

  void GetUuid(nsString& aUuidStr) const
  {
    aUuidStr = mUuidStr;
  }

  /****************************************************************************
   * Others
   ***************************************************************************/
  void Notify(const BluetoothSignal& aData); // BluetoothSignalObserver

  nsPIDOMWindow* GetParentObject() const
  {
     return mOwner;
  }

  virtual JSObject* WrapObject(JSContext* aCx,
                               JS::Handle<JSObject*> aGivenProto) override;

  BluetoothGattDescriptor(nsPIDOMWindow* aOwner,
                          BluetoothGattCharacteristic* aCharacteristic,
                          const BluetoothGattId& aDescriptorId);

private:
  ~BluetoothGattDescriptor();

  /****************************************************************************
   * Variables
   ***************************************************************************/
  nsCOMPtr<nsPIDOMWindow> mOwner;

  /**
   * Characteristic that this descriptor belongs to.
   */
  nsRefPtr<BluetoothGattCharacteristic> mCharacteristic;

  /**
   * GattId of this GATT descriptor which contains
   * 1) mUuid: UUID of this descriptor in byte array format.
   * 2) mInstanceId: Instance id of this descriptor.
   */
  BluetoothGattId mDescriptorId;

  /**
   * UUID string of this GATT descriptor.
   */
  nsString mUuidStr;
};

END_BLUETOOTH_NAMESPACE

#endif
