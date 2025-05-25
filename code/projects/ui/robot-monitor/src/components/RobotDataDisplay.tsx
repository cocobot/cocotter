import React, { useState, useEffect, useMemo, useCallback } from 'react';
import { Box, Paper, Typography } from '@mui/material';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
  ScatterChart,
  Scatter,
} from 'recharts';
import { bluetoothService } from '../services/WebBluetoothService';

interface Position {
  x: number;      // mm
  y: number;      // mm
  theta: number;  // rad
  distanceSpeed: number;  // mm/s
  angleSpeed: number;    // rad/s
}

interface MotorDebug {
  timestamp: number;     // ms
  leftTick: number;      // encoder ticks
  rightTick: number;     // encoder ticks
  leftPwm: number;       // PWM value
  rightPwm: number;      // PWM value
}

const MAX_DATA_POINTS = 5000;
const CHART_UPDATE_INTERVAL = 100; // ms

export const RobotDataDisplay: React.FC = () => {
  const [positionData, setPositionData] = useState<Position[]>([]);
  const [motorDebugData, setMotorDebugData] = useState<MotorDebug[]>([]);
  const [batteryLevel, setBatteryLevel] = useState<number>(0);
  const [timeData, setTimeData] = useState<number[]>([]);
  const [currentPosition, setCurrentPosition] = useState<Position | null>(null);
  const [currentMotorDebug, setCurrentMotorDebug] = useState<MotorDebug | null>(null);
  const [lastChartUpdate, setLastChartUpdate] = useState<number>(0);

  const handlePosition = useCallback((position: Position) => {
    setCurrentPosition(position);
    
    const now = Date.now();
    if (now - lastChartUpdate >= CHART_UPDATE_INTERVAL) {
      setPositionData(prev => {
        const newData = [...prev, position];
        return newData.slice(-MAX_DATA_POINTS);
      });
      setTimeData(prev => {
        const newData = [...prev, now];
        return newData.slice(-MAX_DATA_POINTS);
      });
      setLastChartUpdate(now);
    }
  }, [lastChartUpdate]);

  const handleMotorDebug = useCallback((motorDebug: MotorDebug) => {
    setCurrentMotorDebug(motorDebug);
    
    const now = Date.now();
    if (now - lastChartUpdate >= CHART_UPDATE_INTERVAL) {
      setMotorDebugData(prev => {
        const newData = [...prev, motorDebug];
        return newData.slice(-MAX_DATA_POINTS);
      });
      setLastChartUpdate(now);
    }
  }, [lastChartUpdate]);

  const handleBattery = useCallback((level: number) => {
    setBatteryLevel(level);
  }, []);

  useEffect(() => {
    bluetoothService.on('position', handlePosition);
    bluetoothService.on('motorDebug', handleMotorDebug);
    bluetoothService.on('battery', handleBattery);

    return () => {
      bluetoothService.off('position', handlePosition);
      bluetoothService.off('motorDebug', handleMotorDebug);
      bluetoothService.off('battery', handleBattery);
    };
  }, [handlePosition, handleMotorDebug, handleBattery]);

  const positionChartData = useMemo(() => 
    positionData.map((data, index) => ({
      time: timeData[index] ? (timeData[index] - timeData[0]) / 1000 : 0,
      x: data.x,
      y: data.y,
      theta: data.theta * 180 / Math.PI, // Convert to degrees
    })), [positionData, timeData]);

  const motorChartData = useMemo(() => 
    motorDebugData.map(data => ({
      time: data.timestamp / 1000, // Convert to seconds
      leftTick: data.leftTick,
      rightTick: data.rightTick,
      leftPwm: data.leftPwm,
      rightPwm: data.rightPwm,
    })), [motorDebugData]);

  return (
    <Box sx={{ p: 2, display: 'flex', flexDirection: 'column', gap: 2 }}>
      {/* Status Information */}
      <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2 }}>
        {/* Battery Level */}
        <Paper sx={{ p: 2, flex: '1 1 200px' }}>
          <Typography variant="h6" gutterBottom>
            Battery Level: {batteryLevel}%
          </Typography>
        </Paper>

        {/* Current Position */}
        <Paper sx={{ p: 2, flex: '1 1 200px' }}>
          <Typography variant="h6" gutterBottom>
            Current Position
          </Typography>
          {currentPosition && (
            <>
              <Typography>X: {currentPosition.x.toFixed(2)} mm</Typography>
              <Typography>Y: {currentPosition.y.toFixed(2)} mm</Typography>
              <Typography>θ: {(currentPosition.theta * 180 / Math.PI).toFixed(2)}°</Typography>
            </>
          )}
        </Paper>

        {/* Current Motor Debug */}
        <Paper sx={{ p: 2, flex: '1 1 200px' }}>
          <Typography variant="h6" gutterBottom>
            Motor Status
          </Typography>
          {currentMotorDebug && (
            <>
              <Typography>Left Encoder: {currentMotorDebug.leftTick}</Typography>
              <Typography>Right Encoder: {currentMotorDebug.rightTick}</Typography>
              <Typography>Left PWM: {currentMotorDebug.leftPwm}</Typography>
              <Typography>Right PWM: {currentMotorDebug.rightPwm}</Typography>
            </>
          )}
        </Paper>
      </Box>

      {/* Charts Container */}
      <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2 }}>
        {/* XY Position Chart */}
        <Box sx={{ flex: '1 1 400px', minWidth: 0 }}>
          <Paper sx={{ p: 2 }}>
            <Typography variant="h6" gutterBottom>
              XY Position
            </Typography>
            <ResponsiveContainer width="100%" height={300}>
              <ScatterChart>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis type="number" dataKey="x" name="X (mm)" />
                <YAxis type="number" dataKey="y" name="Y (mm)" />
                <Tooltip />
                <Legend />
                <Scatter data={positionChartData} fill="#8884d8" name="Position" />
              </ScatterChart>
            </ResponsiveContainer>
          </Paper>
        </Box>

        {/* Position Chart */}
        <Box sx={{ flex: '1 1 400px', minWidth: 0 }}>
          <Paper sx={{ p: 2 }}>
            <Typography variant="h6" gutterBottom>
              Position vs Time
            </Typography>
            <ResponsiveContainer width="100%" height={300}>
              <LineChart data={positionChartData}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" label={{ value: 'Time (s)', position: 'bottom' }} />
                <YAxis />
                <Tooltip />
                <Legend />
                <Line type="monotone" dataKey="x" stroke="#8884d8" name="X (mm)" />
                <Line type="monotone" dataKey="y" stroke="#82ca9d" name="Y (mm)" />
                <Line type="monotone" dataKey="theta" stroke="#ffc658" name="Theta (deg)" />
              </LineChart>
            </ResponsiveContainer>
          </Paper>
        </Box>

        {/* Motor Debug Chart */}
        <Box sx={{ flex: '1 1 100%', minWidth: 0 }}>
          <Paper sx={{ p: 2 }}>
            <Typography variant="h6" gutterBottom>
              Motor Debug
            </Typography>
            <ResponsiveContainer width="100%" height={300}>
              <LineChart data={motorChartData}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis dataKey="time" label={{ value: 'Time (s)', position: 'bottom' }} />
                <YAxis />
                <Tooltip />
                <Legend />
                <Line type="monotone" dataKey="leftTick" stroke="#8884d8" name="Left Encoder" />
                <Line type="monotone" dataKey="rightTick" stroke="#82ca9d" name="Right Encoder" />
                <Line type="monotone" dataKey="leftPwm" stroke="#ffc658" name="Left PWM" />
                <Line type="monotone" dataKey="rightPwm" stroke="#ff7300" name="Right PWM" />
              </LineChart>
            </ResponsiveContainer>
          </Paper>
        </Box>
      </Box>
    </Box>
  );
}; 