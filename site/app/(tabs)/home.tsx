import { View, StyleSheet, TouchableOpacity } from 'react-native';
import { FontAwesome } from '@expo/vector-icons';
import { ThemedText } from '@/components/ThemedText';
import { ThemedView } from '@/components/ThemedView';
import { useDroneStore } from '@/utils/droneConnection';

export default function HomeScreen() {
  const { isConnected, telemetryData, connect, disconnect } = useDroneStore();

  return (
    <ThemedView style={styles.container}>
      <View style={styles.header}>
        <ThemedText style={styles.headerText}>
          Status: {isConnected ? 'Connected' : 'Disconnected'}
        </ThemedText>
        <TouchableOpacity 
          onPress={isConnected ? disconnect : connect}
          style={[styles.connectButton, isConnected ? styles.connected : styles.disconnected]}
        >
          <FontAwesome name="power-off" size={24} color={isConnected ? '#4CAF50' : '#F44336'} />
        </TouchableOpacity>
      </View>

      <View style={styles.grid}>
        <TelemetryCard title="Altitude" value={`${telemetryData.altitude}m`} />
        <TelemetryCard title="Speed" value={`${telemetryData.speed}km/h`} />
        <TelemetryCard title="Battery" value={`${telemetryData.battery}%`} />
        <TelemetryCard title="GPS" value={`${telemetryData.gpsSatellites} sats`} />
        <TelemetryCard title="Signal" value={`${telemetryData.signalStrength}%`} />
        <TelemetryCard title="Flight Time" value={`${telemetryData.flightTime}s`} />
      </View>
    </ThemedView>
  );
}

function TelemetryCard({ title, value }: { title: string; value: string }) {
  return (
    <View style={styles.card}>
      <ThemedText style={styles.cardTitle}>{title}</ThemedText>
      <ThemedText style={styles.cardValue}>{value}</ThemedText>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    padding: 16,
  },
  grid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 16,
  },
  card: {
    width: '47%',
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#f5f5f5',
  },
  cardTitle: {
    fontSize: 16,
    fontWeight: 'bold',
  },
  cardValue: {
    fontSize: 24,
    marginTop: 8,
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 16,
  },
  headerText: {
    fontSize: 18,
    fontWeight: 'bold',
  },
  connectButton: {
    padding: 12,
    borderRadius: 8,
  },
  connected: {
    backgroundColor: '#E8F5E9',
  },
  disconnected: {
    backgroundColor: '#FFEBEE',
  },
});