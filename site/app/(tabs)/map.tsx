
import { StyleSheet } from 'react-native';
import MapView, { Marker } from 'react-native-maps';
import { useDroneStore } from '@/utils/droneConnection';

export default function MapScreen() {
  const { telemetryData, isConnected } = useDroneStore();

  return (
    <MapView style={styles.map}>
      {isConnected && (
        <Marker
          coordinate={{
            latitude: telemetryData.latitude,
            longitude: telemetryData.longitude,
          }}
          title="Drone"
          description={`Alt: ${telemetryData.altitude}m`}
        />
      )}
    </MapView>
  );
}

const styles = StyleSheet.create({
  map: {
    flex: 1,
  },
});