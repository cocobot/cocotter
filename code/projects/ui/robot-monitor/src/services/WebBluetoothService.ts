declare global {
  interface Navigator {
    bluetooth: {
      requestDevice(options: { 
        acceptAllDevices?: boolean;
        filters?: { services: string[] }[];
        optionalServices?: string[];
      }): Promise<BluetoothDevice>;
    };
  }
}

// Web Bluetooth API types
interface BluetoothDevice {
  gatt?: BluetoothRemoteGATTServer;
}

interface BluetoothRemoteGATTServer {
  connected: boolean;
  connect(): Promise<BluetoothRemoteGATTServer>;
  disconnect(): void;
  getPrimaryService(service: string): Promise<BluetoothRemoteGATTService>;
}

interface BluetoothRemoteGATTService {
  getCharacteristic(characteristic: string): Promise<BluetoothRemoteGATTCharacteristic>;
}

interface BluetoothRemoteGATTCharacteristic {
  value: DataView | null;
  startNotifications(): Promise<BluetoothRemoteGATTCharacteristic>;
  stopNotifications(): Promise<BluetoothRemoteGATTCharacteristic>;
  addEventListener(type: string, listener: EventListener): void;
  removeEventListener(type: string, listener: EventListener): void;
  writeValue(value: ArrayBuffer): Promise<void>;
}

// BLE Service and Characteristic UUIDs
const BATTERY_SERVICE_UUID = '0000180f-0000-1000-8000-00805f9b34fb';
const BATTERY_LEVEL_CHAR_UUID = '00002a19-0000-1000-8000-00805f9b34fb';

const PAMI_SERVICE_UUID = 'c10e0000-5a32-42a0-886b-cf9d57a5fd4a';
//const PAMI_OVERRIDE_PWM_CHAR_UUID = 'c10e0001-5a32-42a0-886b-cf9d57a5fd4a';
const PAMI_POSITION_CHAR_UUID = 'c10e0002-5a32-42a0-886b-cf9d57a5fd4a';
const PAMI_MOTOR_DEBUG_CHAR_UUID = 'c10e0003-5a32-42a0-886b-cf9d57a5fd4a';
const PAMI_OVERRIDE_MOTOR_CHAR_UUID = 'c10e0004-5a32-42a0-886b-cf9d57a5fd4a';

// Custom EventEmitter implementation
class EventEmitter {
  private events: { [key: string]: Function[] } = {};

  on(event: string, listener: (...args: any[]) => void): this {
    if (!this.events[event]) {
      this.events[event] = [];
    }
    this.events[event].push(listener);
    return this;
  }

  off(event: string, listener: (...args: any[]) => void): this {
    if (!this.events[event]) return this;
    this.events[event] = this.events[event].filter(l => l !== listener);
    return this;
  }

  emit(event: string, ...args: any[]): boolean {
    if (!this.events[event]) return false;
    this.events[event].forEach(listener => listener(...args));
    return true;
  }
}

export interface RobotData {
  position: {
    x: number;      // mm
    y: number;      // mm
    theta: number;  // rad
    distanceSpeed: number;  // mm/s
    angleSpeed: number;    // rad/s
  };
  motorDebug: {
    timestamp: number;     // ms
    leftTick: number;      // encoder ticks
    rightTick: number;     // encoder ticks
    leftPwm: number;       // PWM value
    rightPwm: number;      // PWM value
  };
  batteryLevel: number;    // percentage
}

class WebBluetoothService extends EventEmitter {
  private device: BluetoothDevice | null = null;
  private batteryService: BluetoothRemoteGATTService | null = null;
  private pamiService: BluetoothRemoteGATTService | null = null;
  private batteryLevelChar: BluetoothRemoteGATTCharacteristic | null = null;
  private positionChar: BluetoothRemoteGATTCharacteristic | null = null;
  private motorDebugChar: BluetoothRemoteGATTCharacteristic | null = null;
  private isConnected: boolean = false;

  constructor() {
    super();
  }

  // Expose EventEmitter methods
  public on(event: string, listener: (...args: any[]) => void): this {
    return super.on(event, listener);
  }

  public off(event: string, listener: (...args: any[]) => void): this {
    return super.off(event, listener);
  }

  public emit(event: string, ...args: any[]): boolean {
    return super.emit(event, ...args);
  }

  async connect(): Promise<void> {
    try {
      this.device = await navigator.bluetooth.requestDevice({
        acceptAllDevices: true,
        optionalServices: [BATTERY_SERVICE_UUID, PAMI_SERVICE_UUID],  
      });

      console.log("T1")
      const server = await this.device.gatt?.connect();
      if (!server) throw new Error('Failed to connect to GATT server');

      console.log("T2")
      // Get services
      this.batteryService = await server.getPrimaryService(BATTERY_SERVICE_UUID);
      this.pamiService = await server.getPrimaryService(PAMI_SERVICE_UUID);

      console.log("T3")

      // Get characteristics
      this.batteryLevelChar = await this.batteryService.getCharacteristic(BATTERY_LEVEL_CHAR_UUID);
      this.positionChar = await this.pamiService.getCharacteristic(PAMI_POSITION_CHAR_UUID);
      this.motorDebugChar = await this.pamiService.getCharacteristic(PAMI_MOTOR_DEBUG_CHAR_UUID);

      console.log("T4")

      // Start notifications
      await this.batteryLevelChar.startNotifications();
      await this.positionChar.startNotifications();
      await this.motorDebugChar.startNotifications();

      console.log("T5")

      // Add event listeners
      this.batteryLevelChar.addEventListener('characteristicvaluechanged', this.handleBatteryData.bind(this));
      this.positionChar.addEventListener('characteristicvaluechanged', this.handlePositionData.bind(this));
      this.motorDebugChar.addEventListener('characteristicvaluechanged', this.handleMotorDebugData.bind(this));

      console.log("T6")

      this.isConnected = true;
      this.emit('connected');
    } catch (error) {
      console.error('Connection error:', error);
      throw error;
    }
  }

  disconnect(): void {
    if (this.batteryLevelChar) {
      this.batteryLevelChar.removeEventListener('characteristicvaluechanged', this.handleBatteryData.bind(this));
      this.batteryLevelChar.stopNotifications();
    }
    if (this.positionChar) {
      this.positionChar.removeEventListener('characteristicvaluechanged', this.handlePositionData.bind(this));
      this.positionChar.stopNotifications();
    }
    if (this.motorDebugChar) {
      this.motorDebugChar.removeEventListener('characteristicvaluechanged', this.handleMotorDebugData.bind(this));
      this.motorDebugChar.stopNotifications();
    }
    if (this.device?.gatt?.connected) {
      this.device.gatt.disconnect();
    }
    this.isConnected = false;
    this.emit('disconnected');
  }

  private handleBatteryData(event: Event): void {
    const value = (event.target as unknown as BluetoothRemoteGATTCharacteristic).value;
    if (value) {
      const batteryLevel = value.getUint8(0);
      this.emit('battery', batteryLevel);
    }
  }

  private handlePositionData(event: Event): void {
    const value = (event.target as unknown as BluetoothRemoteGATTCharacteristic).value;
    if (value) {
      const position = {
        x: value.getFloat32(0, true),           // mm
        y: value.getFloat32(4, true),           // mm
        theta: value.getFloat32(8, true),       // rad
        distanceSpeed: value.getFloat32(12, true), // mm/s
        angleSpeed: value.getFloat32(16, true),    // rad/s
      };
      this.emit('position', position);
    }
  }

  private handleMotorDebugData(event: Event): void {
    const value = (event.target as unknown as BluetoothRemoteGATTCharacteristic).value;
    if (value) {
      const motorDebug = {
        timestamp: value.getUint16(0, true),    // ms
        leftTick: value.getInt32(2, true),      // encoder ticks
        rightTick: value.getInt32(6, true),     // encoder ticks
        leftPwm: value.getInt16(10, true),      // PWM value
        rightPwm: value.getInt16(12, true),     // PWM value
      };
      this.emit('motorDebug', motorDebug);
    }
  }

  async sendMotorOverride(afterFilter: boolean, left: number, right: number): Promise<void> {
    if (!this.pamiService) throw new Error('Not connected');
    
    const overrideChar = await this.pamiService.getCharacteristic(PAMI_OVERRIDE_MOTOR_CHAR_UUID);
    const buffer = new ArrayBuffer(5);
    const view = new DataView(buffer);
    
    view.setUint8(0, afterFilter ? 1 : 0);
    view.setInt16(1, left, true);
    view.setInt16(3, right, true);
    
    await overrideChar.writeValue(buffer);
  }

  isDeviceConnected(): boolean {
    return this.isConnected;
  }
}

export const bluetoothService = new WebBluetoothService(); 