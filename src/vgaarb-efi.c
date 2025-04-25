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

// Function prototype for recursive bridge programming - Updated to use device handle
EFI_STATUS
EnableVgaDecodingRecursively(
    IN EFI_HANDLE DeviceHandle,
    IN BOOLEAN Enable
);

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
    UINT16 BridgeControl;
    
    // Read current Bridge Control register
    Status = uefi_call_wrapper(PciIo->Pci.Read, 5,
        PciIo,
        EfiPciIoWidthUint16,
        PCI_BRIDGE_CONTROL,
        1,
        &BridgeControl
    );
    
    if (EFI_ERROR(Status)) {
        return Status;
    }
    
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

// Recursively enable VGA decoding up through the parent bridges
// Updated to use EFI_HANDLE instead of PciIo directly
EFI_STATUS
EnableVgaDecodingRecursively(
    IN EFI_HANDLE DeviceHandle,
    IN BOOLEAN Enable
)
{
    EFI_STATUS Status;
    UINT16 Command;
    EFI_DEVICE_PATH_PROTOCOL *DevicePath;
    EFI_DEVICE_PATH_PROTOCOL *ParentPath;
    EFI_PCI_IO_PROTOCOL *PciIo;
    EFI_HANDLE ParentHandle;
    EFI_GUID PciIoGuid = EFI_PCI_IO_PROTOCOL_GUID;
    EFI_GUID DevicePathGuid = EFI_DEVICE_PATH_PROTOCOL_GUID;
    
    // Get the PCI IO protocol for this handle
    Status = uefi_call_wrapper(BS->HandleProtocol, 3,
        DeviceHandle,
        &PciIoGuid,
        (VOID **)&PciIo
    );
    
    if (EFI_ERROR(Status) || PciIo == NULL) {
        return Status;
    }
    
    // Enable I/O, Memory and Bus Master on the device itself
    Status = uefi_call_wrapper(PciIo->Pci.Read, 5,
        PciIo,
        EfiPciIoWidthUint16,
        PCI_COMMAND,
        1,
        &Command
    );
    
    if (EFI_ERROR(Status)) {
        return Status;
    }
    
    if (Enable) {
        Command |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
    }
    
    Status = uefi_call_wrapper(PciIo->Pci.Write, 5,
        PciIo,
        EfiPciIoWidthUint16,
        PCI_COMMAND,
        1,
        &Command
    );
    
    if (EFI_ERROR(Status)) {
        return Status;
    }
    
    // Get the device path of this device
    Status = uefi_call_wrapper(BS->HandleProtocol, 3,
        DeviceHandle,
        &DevicePathGuid,
        (VOID **)&DevicePath
    );
    
    if (EFI_ERROR(Status) || DevicePath == NULL) {
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
            Status = EnableVgaDecodingRecursively(ParentHandle, Enable);
        }
    }
    
    return EFI_SUCCESS;
}

// Function to discover and enumerate all VGA devices in the system
EFI_STATUS
DiscoverAndEnableVgaDevices(VOID)
{
    EFI_STATUS Status;
    EFI_HANDLE *HandleBuffer;
    UINTN HandleCount;
    UINTN Index;
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
        
        // Check if this is a VGA device
        if (IsVgaDevice(PciIo)) {
            UINTN Segment, Bus, Device, Function;
            
            // Get device location
            Status = uefi_call_wrapper(PciIo->GetLocation, 5,
                PciIo,
                &Segment,
                &Bus,
                &Device,
                &Function
            );
            
            if (!EFI_ERROR(Status)) {
                Print(L"Found VGA device at Seg:%d Bus:%d Device:%d Function:%d\n", 
                     Segment, Bus, Device, Function);
                
                // Enable VGA decoding recursively - Now pass the handle instead of PciIo
                Status = EnableVgaDecodingRecursively(HandleBuffer[Index], TRUE);
                
                if (EFI_ERROR(Status)) {
                    Print(L"Failed to enable VGA decoding: %r\n", Status);
                } else {
                    Print(L"Successfully enabled VGA decoding for device\n");
                }
            }
        }
    }
    
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
