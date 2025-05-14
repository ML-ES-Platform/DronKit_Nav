import { create, StateCreator } from 'zustand';

interface TelemetryData {
  altitude: number;
  speed: number;
  battery: number;
  gpsSatellites: number;
  signalStrength: number;
  flightTime: number;
  latitude: number;
  longitude: number;
}

interface DroneState {
  isConnected: boolean;
  telemetryData: TelemetryData;
  connect: () => Promise<void>;
  disconnect: () => void;
}

const createStore: StateCreator<DroneState> = (set) => ({
  isConnected: false,
  telemetryData: {
    altitude: 0,
    speed: 0,
    battery: 100,
    gpsSatellites: 0,
    signalStrength: 0,
    flightTime: 0,
    latitude: 0,
    longitude: 0,
  },
  connect: async () => {
    set({ isConnected: true });
  },
  disconnect: () => {
    set({ isConnected: false });
  },
});

export const useDroneStore = create<DroneState>(createStore);