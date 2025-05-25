# Robot Monitor

A React application for monitoring robot data via WebBluetooth. This application allows you to connect to a robot and display real-time data such as position, encoder values, and PID control information.

## Features

- WebBluetooth connection to robot
- Real-time data visualization
- Position tracking (X, Y, Theta)
- Encoder values monitoring
- PID control visualization
- Responsive design
- Dark theme UI

## Prerequisites

- Modern web browser with WebBluetooth support (Chrome, Edge, or Opera)
- Node.js (v14 or higher)
- npm (v6 or higher)

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd robot-monitor
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm run dev
```

4. Open your browser and navigate to `http://localhost:5173`

## Usage

1. Click the "Connect to Robot" button in the top-right corner
2. Select your robot from the Bluetooth device list
3. Once connected, the application will start displaying real-time data in the charts
4. The charts show:
   - Position (X, Y, Theta)
   - Encoder values (Left and Right)
   - PID control data (Setpoint, Current, Error)

## Development

The project is built with:
- Vite
- React
- TypeScript
- Material-UI
- Recharts

## Building for Production

To create a production build:

```bash
npm run build
```

The built files will be in the `dist` directory.

## Notes

- WebBluetooth is only supported in secure contexts (HTTPS or localhost)
- Make sure your robot is compatible with the WebBluetooth API
- The application expects specific data format from the robot (see RobotData interface in WebBluetoothService.ts)
