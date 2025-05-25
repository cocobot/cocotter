import React, { useState, useEffect } from 'react';
import { Button, CircularProgress } from '@mui/material';
import { bluetoothService } from '../services/WebBluetoothService';

export const ConnectionButton: React.FC = () => {
  const [isConnecting, setIsConnecting] = useState(false);
  const [isConnected, setIsConnected] = useState(false);

  useEffect(() => {
    const handleConnected = () => {
      setIsConnected(true);
      setIsConnecting(false);
    };

    const handleDisconnected = () => {
      setIsConnected(false);
      setIsConnecting(false);
    };

    bluetoothService.on('connected', handleConnected);
    bluetoothService.on('disconnected', handleDisconnected);

    return () => {
      bluetoothService.off('connected', handleConnected);
      bluetoothService.off('disconnected', handleDisconnected);
    };
  }, []);

  const handleClick = async () => {
    if (isConnected) {
      bluetoothService.disconnect();
    } else {
      setIsConnecting(true);
      try {
        await bluetoothService.connect();
      } catch (error) {
        console.error('Failed to connect:', error);
        setIsConnecting(false);
      }
    }
  };

  return (
    <Button
      variant="contained"
      color={isConnected ? "error" : "primary"}
      onClick={handleClick}
      disabled={isConnecting}
      startIcon={isConnecting ? <CircularProgress size={20} /> : null}
    >
      {isConnecting ? 'Connecting...' : isConnected ? 'Disconnect' : 'Connect to Robot'}
    </Button>
  );
}; 