#include <efi.h>
#include <efilib.h>
#include <efipciio.h>
#include <efidevp.h>

// PCI Configuration Space register offsets
#define PCI_COMMAND             0x04    // Command Register
#define PCI_BRIDGE_CONTROL      0x3E    // Bridge Control Register offset for PCI-to-PCI bridges

// Command Register bits
#define PCI_COMMAND_IO          0x0001  // I/O Space Enable
#define PCI_COMMAND_MEMORY      0x0002  // Memory Space Enable
#define PCI_COMMAND_MASTER      0x0004  // Bus Master Enable

// Bridge Control Register bits
#define PCI_BRIDGE_CTL_VGA      0x0008  // VGA Enable bit for PCI-to-PCI bridges

// PCI Class Codes
#define PCI_CLASS_DISPLAY       0x03    // Display controller
#define PCI_CLASS_BRIDGE        0x06    // Bridge device
#define PCI_SUBCLASS_PCI_BRIDGE 0x04    // PCI-to-PCI Bridge

// PCI header types
#define PCI_HEADER_TYPE_NORMAL  0x00    // Normal PCI device
#define PCI_HEADER_TYPE_BRIDGE  0x01    // PCI-to-PCI bridge
#define PCI_HEADER_TYPE_MASK    0x7F    // Mask for header type

#define MAX_VGA_DEVICES 10

CHAR16 *
PciIoToSBDFStr(
    IN EFI_PCI_IO_PROTOCOL *PciIo
)
{
    EFI_STATUS Status;
    UINTN Segment, Bus, Device, Function;
            
    // Get device location
    Status = uefi_call_wrapper(PciIo->GetLocation, 5,
        PciIo,
        &Segment,
        &Bus,
        &Device,
        &Function
    );
    if (EFI_ERROR(Status)) {
        return L"UNKNOWN";
    }
    // Format into a string
    CHAR16 *Buffer = AllocateZeroPool(32 * sizeof(CHAR16));
    if (Buffer == NULL) {
        return L"UNKNOWN";
    }

    SPrint(Buffer, 32 * sizeof(CHAR16), L"%04X:%02X:%02X.%01X", 
        (UINT16)Segment, (UINT8)Bus, (UINT8)Device, (UINT8)Function);
    
    return Buffer;
}

// Function to determine if a device is a VGA-compatible device
BOOLEAN
IsVgaDevice(
    IN EFI_PCI_IO_PROTOCOL *PciIo
)
{
    UINT32 ClassCode;
    EFI_STATUS Status;
    
    // Read Class Code (includes class, subclass, and prog interface)
    Status = uefi_call_wrapper(PciIo->Pci.Read, 5,
        PciIo,
        EfiPciIoWidthUint32,
        0x08,           // Class code is at offset 0x08
        1,
        &ClassCode
    );
    
    if (EFI_ERROR(Status)) {
        Print(L"IsVgaDevice: Failed to read class code: %r for %s\n", Status,
            PciIoToSBDFStr(PciIo));
        return FALSE;
    }
    
    // Extract Class and Subclass (shift right to isolate class code)
    UINT8 Class = (ClassCode >> 24) & 0xFF;
    
    // Check if this is a display/VGA controller (class 0x03)
    return (Class == PCI_CLASS_DISPLAY);
}

// Function to enable VGA decoding on a PCI-to-PCI bridge
EFI_STATUS
EnableVgaOnBridge(
    IN EFI_PCI_IO_PROTOCOL *PciIo,
    IN BOOLEAN Enable
)
{
    EFI_STATUS Status;
    UINT16 BridgeControl, BridgeControlOrig;
    
    // Read current Bridge Control register
    Status = uefi_call_wrapper(PciIo->Pci.Read, 5,
        PciIo,
        EfiPciIoWidthUint16,
        PCI_BRIDGE_CONTROL,
        1,
        &BridgeControl
    );
    
    if (EFI_ERROR(Status)) {
        Print(L"EnableVgaOnBridge: Failed to read Bridge Control: %r\n", Status);
        return Status;
    }

    BridgeControlOrig = BridgeControl;

    // Modify VGA Enable bit
    if (Enable) {
        BridgeControl |= PCI_BRIDGE_CTL_VGA;  // Enable VGA
    } else {
        BridgeControl &= ~PCI_BRIDGE_CTL_VGA; // Disable VGA
    }
    
    // Write back to Bridge Control register
    Status = uefi_call_wrapper(PciIo->Pci.Write, 5,
        PciIo,
        EfiPciIoWidthUint16,
        PCI_BRIDGE_CONTROL,
        1,
        &BridgeControl
    );

    if (EFI_ERROR(Status)) {
        Print(L"EnableVgaOnBridge: Failed to write Bridge Control: %r\n", Status);
        return Status;
    }

    // Read back to verify
    Status = uefi_call_wrapper(PciIo->Pci.Read, 5,
        PciIo,
        EfiPciIoWidthUint16,
        PCI_BRIDGE_CONTROL,
        1,
        &BridgeControl
    );

    if (EFI_ERROR(Status)) {
        Print(L"EnableVgaOnBridge: Failed to read back Bridge Control: %r\n", Status);
        return Status;
    }

    Print(L"EnableVgaOnBridge: %s, Enable = %d, BridgeControl: %04X -> %04X\n", 
        PciIoToSBDFStr(PciIo), !!Enable, BridgeControlOrig, BridgeControl);
    
    return Status;
}

// Function to check if a device is a PCI bridge
BOOLEAN
IsPciBridge(
    IN EFI_PCI_IO_PROTOCOL *PciIo
)
{
    UINT32 ClassCode;
    UINT8 HeaderType;
    EFI_STATUS Status;
    
    // Read header type
    Status = uefi_call_wrapper(PciIo->Pci.Read, 5,
        PciIo,
        EfiPciIoWidthUint8,
        0x0E,           // Header Type offset
        1,
        &HeaderType
    );
    
    if (EFI_ERROR(Status)) {
        Print(L"IsPciBridge: Failed to read header type: %r\n", Status);
        return FALSE;
    }
    
    // Get class code
    Status = uefi_call_wrapper(PciIo->Pci.Read, 5,
        PciIo,
        EfiPciIoWidthUint32,
        0x08,           // Class code offset
        1,
        &ClassCode
    );
    
    if (EFI_ERROR(Status)) {
        Print(L"IsPciBridge: Failed to read class code: %r\n", Status);
        return FALSE;
    }
    
    // Extract Class and Subclass
    UINT8 Class = (ClassCode >> 24) & 0xFF;
    UINT8 SubClass = (ClassCode >> 16) & 0xFF;
    
    // Check if this is a PCI-to-PCI bridge
    return ((HeaderType & PCI_HEADER_TYPE_MASK) == PCI_HEADER_TYPE_BRIDGE && 
            Class == PCI_CLASS_BRIDGE && 
            SubClass == PCI_SUBCLASS_PCI_BRIDGE);
}

CHAR16 *
DeviceHandleToPathStr(
    IN EFI_HANDLE DeviceHandle
)
{
    CHAR16 *UNKSTR = L"UNKNOWN";
    CHAR16 *DevicePathStr = UNKSTR;
    EFI_DEVICE_PATH *DevicePath = NULL;
    
    DevicePath = DevicePathFromHandle(DeviceHandle);
    if (DevicePath == NULL) {
        return UNKSTR;
    }
    
    // Convert device path to string
    DevicePathStr = DevicePathToStr(DevicePath);
    if (DevicePathStr == NULL) {
        return UNKSTR;
    }
    
    return DevicePathStr;
}

// Recursively enable VGA decoding up through the parent bridges
EFI_STATUS
EnableVgaDecodingRecursive(
    IN EFI_HANDLE DeviceHandle,
    IN BOOLEAN Enable
)
{
    EFI_STATUS Status;
    EFI_DEVICE_PATH_PROTOCOL *DevicePath;
    EFI_DEVICE_PATH_PROTOCOL *ParentPath;
    EFI_PCI_IO_PROTOCOL *PciIo;
    EFI_HANDLE ParentHandle;
    EFI_GUID PciIoGuid = EFI_PCI_IO_PROTOCOL_GUID;
    EFI_GUID DevicePathGuid = EFI_DEVICE_PATH_PROTOCOL_GUID;

    Print(L"EnableVgaDecodingRecursive: %s, Enable=%d\n", 
          DeviceHandleToPathStr(DeviceHandle), Enable);

    // Get the PCI IO protocol for this handle
    Status = uefi_call_wrapper(BS->HandleProtocol, 3,
        DeviceHandle,
        &PciIoGuid,
        (VOID **)&PciIo
    );
    
    if (EFI_ERROR(Status) || PciIo == NULL) {
        return Status;
    }

    // Get the device path of this device
    Status = uefi_call_wrapper(BS->HandleProtocol, 3,
        DeviceHandle,
        &DevicePathGuid,
        (VOID **)&DevicePath
    );
    
    if (EFI_ERROR(Status) || DevicePath == NULL) {
        Print(L"Failed to get device path: %r\n", Status);
        return Status;
    }
    
    // If this is a bridge, enable VGA decoding on it
    if (IsPciBridge(PciIo)) {
        Status = EnableVgaOnBridge(PciIo, Enable);
        if (EFI_ERROR(Status)) {
            return Status;
        }
    }
    
    // Check if we can get a parent device path
    ParentPath = DevicePath;
    
    // Find the parent bridge by removing the last node from the device path
    while (!IsDevicePathEnd(ParentPath)) {
        EFI_DEVICE_PATH_PROTOCOL *TempPath = ParentPath;
        ParentPath = NextDevicePathNode(ParentPath);
        
        // If we're at the end, use the previous path as our parent
        if (IsDevicePathEnd(ParentPath)) {
            // Set end-of-path node
            SetDevicePathEndNode(TempPath);
            ParentPath = DevicePath;
            break;
        }
    }
    
    // If we have a valid parent path (different from our path)
    if (ParentPath != DevicePath && !IsDevicePathEnd(ParentPath)) {
        // Look up the parent handle
        Status = uefi_call_wrapper(BS->LocateDevicePath, 3,
            &PciIoGuid,
            &ParentPath,
            &ParentHandle
        );
        
        if (!EFI_ERROR(Status)) {
            // Recursively enable VGA decoding on the parent
            Status = EnableVgaDecodingRecursive(ParentHandle, Enable);
        }
    }
    
    return EFI_SUCCESS;
}

EFI_STATUS
EnableVgaAttributes(
    IN EFI_PCI_IO_PROTOCOL *PciIo
)
{
    EFI_STATUS Status;
    UINT64 Supported;
    UINT64 Attributes = 0;
    BOOLEAN unsupported = FALSE;
    
    // Read the current command register
    Status = uefi_call_wrapper(PciIo->Attributes, 5,
        PciIo,
        EfiPciIoAttributeOperationSupported,
        0,
        &Supported
    );

    if (EFI_ERROR(Status)) {
        Print(L"EnableVgaAttributes: Failed to get supported attributes: %r\n", Status);
        return Status;
    }

    Attributes = Supported & (EFI_PCI_IO_ATTRIBUTE_VGA_IO | EFI_PCI_IO_ATTRIBUTE_VGA_IO_16);

    if (Attributes == 0) {
        Print(L"EnableVgaAttributes: No VGA attributes found\n");
        unsupported = TRUE;
    } else if (Attributes == (EFI_PCI_IO_ATTRIBUTE_VGA_IO | EFI_PCI_IO_ATTRIBUTE_VGA_IO_16)) {
        Print(L"EnableVgaAttributes: VGA attributes support\n");
        Attributes = EFI_PCI_IO_ATTRIBUTE_VGA_IO; // We want to use regular VGA IO
    }

    if (Supported & EFI_PCI_IO_ATTRIBUTE_VGA_MEMORY) {
        Attributes |= EFI_PCI_IO_ATTRIBUTE_VGA_MEMORY;
    } else {
        Print(L"EnableVgaAttributes: No VGA memory support\n");
        unsupported = TRUE;
    }

    // Set the attributes
    Status = uefi_call_wrapper(PciIo->Attributes, 5,
        PciIo,
        EfiPciIoAttributeOperationEnable,
        Attributes,
        NULL
    );

    if (EFI_ERROR(Status)) {
        Print(L"EnableVgaAttributes: Failed to set attributes: %r\n", Status);
        return Status;
    }

    // Read back to verify
    Status = uefi_call_wrapper(PciIo->Attributes, 5,
        PciIo,
        EfiPciIoAttributeOperationGet,
        0,
        &Attributes
    );
    if (EFI_ERROR(Status)) {
        Print(L"EnableVgaAttributes: Failed to read back attributes: %r\n", Status);
        return Status;
    }

    if (!(Attributes & (EFI_PCI_IO_ATTRIBUTE_VGA_IO | EFI_PCI_IO_ATTRIBUTE_VGA_IO_16))) {
        Print(L"EnableVgaAttributes: VGA IO not enabled\n");
        unsupported = TRUE;
    }
    if (!(Attributes & EFI_PCI_IO_ATTRIBUTE_VGA_MEMORY)) {
        Print(L"EnableVgaAttributes: VGA memory not enabled\n");
        unsupported = TRUE;
    }

    if (unsupported) {
        Print(L"EnableVgaAttributes: Unsupported attributes\n");
        return EFI_UNSUPPORTED;
    }

    return EFI_SUCCESS;
}

EFI_STATUS
SelectVgaDevice(
    IN EFI_HANDLE DeviceHandle
)
{
    EFI_STATUS Status;
    EFI_PCI_IO_PROTOCOL *PciIo;
    EFI_GUID PciIoGuid = EFI_PCI_IO_PROTOCOL_GUID;

    // Get the PCI IO protocol for this handle
    Status = uefi_call_wrapper(BS->HandleProtocol, 3,
        DeviceHandle,
        &PciIoGuid,
        (VOID **)&PciIo
    );
    
    if (EFI_ERROR(Status) || PciIo == NULL) {
        return Status;
    }

    Status = EnableVgaAttributes(PciIo);
    if (EFI_ERROR(Status)) {
        Print(L"Failed to enable VGA with attributes: %r\n", Status);
    } else {
        Print(L"VGA attributes enabled successfully\n");
        return EFI_SUCCESS;
    }

    Print(L"Fallback to EnableVgaDecodingRecursive\n");
    // Enable VGA decoding on the device
    Status = EnableVgaDecodingRecursive(DeviceHandle, TRUE);
    
    return Status;
}

// Function to discover and enumerate all VGA devices in the system
EFI_STATUS
DiscoverAndEnableVgaDevices(VOID)
{
    EFI_STATUS Status;
    EFI_HANDLE *HandleBuffer;
    UINTN HandleCount;
    UINTN Index;
    UINTN VGADeviceCount = 0;
    EFI_HANDLE *VGADevices[MAX_VGA_DEVICES];
    EFI_HANDLE *VGADevice;
    EFI_PCI_IO_PROTOCOL *PciIo;
    EFI_GUID PciIoGuid = EFI_PCI_IO_PROTOCOL_GUID;
    
    // Locate all handles with PCI IO protocol
    Status = uefi_call_wrapper(BS->LocateHandleBuffer, 5,
        ByProtocol,
        &PciIoGuid,
        NULL,
        &HandleCount,
        &HandleBuffer
    );
    
    if (EFI_ERROR(Status)) {
        Print(L"Failed to locate PCI devices: %r\n", Status);
        return Status;
    }
    
    Print(L"Found %d PCI devices\n", HandleCount);
    
    // Iterate through all PCI devices
    for (Index = 0; Index < HandleCount; Index++) {
        // Get the PCI IO protocol for this handle
        Status = uefi_call_wrapper(BS->HandleProtocol, 3,
            HandleBuffer[Index],
            &PciIoGuid,
            (VOID **)&PciIo
        );
        
        if (EFI_ERROR(Status)) {
            continue;
        }

        if (IsPciBridge(PciIo)) {
            // Disable VGA decoding on the bridge
            Status = EnableVgaOnBridge(PciIo, FALSE);
            if (EFI_ERROR(Status)) {
                Print(L"Failed to disable VGA on bridge: %r\n", Status);
                continue;
            }
        }
        
        // Check if this is a VGA device
        if (IsVgaDevice(PciIo)) {
            // Store the handle of the VGA device
            if (VGADeviceCount < MAX_VGA_DEVICES) {
                VGADevices[VGADeviceCount++] = HandleBuffer[Index];
            } else {
                Print(L"Warning: More than %d VGA devices found, ignoring extras\n", MAX_VGA_DEVICES);
                break;
            }
        }
    }
    
    if (VGADeviceCount == 0) {
        Print(L"No VGA devices found\n");
        goto efi_main_out;
    }

    VGADevice = VGADevices[0];
    // Enable VGA decoding on the first VGA device

    Print(L"Found %d VGA devices, taking: %s\n", VGADeviceCount, 
        DeviceHandleToPathStr(VGADevice));

    Status = SelectVgaDevice(VGADevice);
    if (EFI_ERROR(Status)) {
        Print(L"Failed to select VGA device: %r\n", Status);
        goto efi_main_out;
    }
    

efi_main_out:
    // Free the handle buffer
    uefi_call_wrapper(BS->FreePool, 1, HandleBuffer);
    
    return EFI_SUCCESS;
}

// Main entry point
EFI_STATUS
EFIAPI
efi_main(EFI_HANDLE ImageHandle, EFI_SYSTEM_TABLE *SystemTable)
{
    EFI_STATUS Status;
    
    // Initialize GNU-EFI library
    InitializeLib(ImageHandle, SystemTable);
    
    Print(L"VGA Arbitration Application\n");
    Print(L"-------------------------\n");
    
    // Discover and enable VGA devices
    Status = DiscoverAndEnableVgaDevices();
    
    if (EFI_ERROR(Status)) {
        Print(L"VGA Arbitration failed: %r\n", Status);
        return Status;
    }
    
    Print(L"VGA Arbitration completed successfully\n");
    
    return EFI_SUCCESS;
}
