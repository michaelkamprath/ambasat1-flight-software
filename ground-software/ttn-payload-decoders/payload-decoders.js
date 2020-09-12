function convertBytesToSignedInt( highByte, lowByte ) {
  var sign = highByte & (1 << 7);
  var x = (((highByte & 0xFF) << 8) | (lowByte & 0xFF));
  if (sign) {
  	// Since javascript uses 32-bit integers, we need to fill
  	// the top two bytes of the integers is 1's to make negative.
    result = 0xFFFF0000 | x;
  } else {
    result = x;
  }
  return result;
}

function DecodeVoltage(bytes) {
	// Voltage Sensor
	if (bytes.length !== 2) {
		return {
			error: "payload length is not correct size",
			port: port,
			length: bytes.length 
		};
	}
	var volts = convertBytesToSignedInt(bytes[0], bytes[1]);
	
	return {
		milli_volts: volts
	};
}

function DecodeLSM9DS1Sensor(bytes) {
	// LSM9DS1 sensor
	if (bytes.length !== 20) {
		return {
			error: "payload length is not correct size",
			port: port,
			length: bytes.length 
		};
	}
	
	var rawAccelX = convertBytesToSignedInt(bytes[0], bytes[1]);
	var rawAccelY = convertBytesToSignedInt(bytes[2], bytes[3]);
	var rawAccelZ = convertBytesToSignedInt(bytes[4], bytes[5]);
	var rawGyroX = convertBytesToSignedInt(bytes[6], bytes[7]);
	var rawGyroY = convertBytesToSignedInt(bytes[8], bytes[9]);
	var rawGyroZ = convertBytesToSignedInt(bytes[10], bytes[11]);
	var rawMagX = convertBytesToSignedInt(bytes[12], bytes[13]);
	var rawMagY = convertBytesToSignedInt(bytes[14], bytes[15]);
	var rawMagZ = convertBytesToSignedInt(bytes[16], bytes[17]);

	var accelSensitivty = 0.0;
	switch (bytes[18]&0x0F) {
		case 0x01:	// ACCELERATION_SENSITIVITY_2G
			accelSensitivty = 0.061;
			break;
		case 0x02:	// ACCELERATION_SENSITIVITY_4G
			accelSensitivty = 0.122;
			break;
		case 0x03:	// ACCELERATION_SENSITIVITY_8G
			accelSensitivty = 0.244;
			break;
		case 0x04:	// ACCELERATION_SENSITIVITY_16G
			accelSensitivty = 0.732;
			break;
		default:
			break;	
	}
	
	var gyroSensitivity = 0.0;
	switch (bytes[18]&0xF0) {
		case 0x10:	// GYRO_SENSITIVITY_245DPS
			gyroSensitivity = 0.00875;
			break;
		case 0x20: 	// GYRO_SENSITIVITY_500DPS
			gyroSensitivity = 0.0175;
			break;
		case 0x30:	// GYRO_SENSITIVITY_2000DPS
			gyroSensitivity = 0.070;
			break;
		default:
			break;
	}
	
	var magneticSensitivity = 0.0;
	switch (bytes[19]&0x0F) {
		case 0x01:	// MAGNETIC_SENSITIVITY_4GAUSS
			magneticSensitivity = 0.14;
			break;
		case 0x02:	// MAGNETIC_SENSITIVITY_8GAUSS
			magneticSensitivity = 0.29;
			break;
		case 0x03:	// MAGNETIC_SENSITIVITY_12GAUSS
			magneticSensitivity = 0.43;
			break;
		case 0x04:	// MAGNETIC_SENSITIVITY_16GAUSS
			magneticSensitivity = 0.58;
			break;
		default:
			break;	
	}
	
	
	return {
		acceleration_x: rawAccelX*accelSensitivty/1000.0,
		acceleration_y: rawAccelY*accelSensitivty/1000.0,
		acceleration_z: rawAccelZ*accelSensitivty/1000.0,
		gyro_x: rawGyroX*gyroSensitivity/1000.0,
		gyro_y: rawGyroY*gyroSensitivity/1000.0,
		gyro_z: rawGyroZ*gyroSensitivity/1000.0,
		magnetic_x: rawMagX*magneticSensitivity/1000.0,
		magnetic_y: rawMagY*magneticSensitivity/1000.0,
		magnetic_z: rawMagZ*magneticSensitivity/1000.0,
	};
}

function DecodeSI1132Sensor(bytes) {
	// LSM9DS1 sensor
	if (bytes.length !== 6) {
		return {
			error: "payload length is not correct size",
			port: port,
			length: bytes.length 
		};
	}

	return {
		uv: convertBytesToSignedInt(bytes[0], bytes[1])/100.0,
		visible: convertBytesToSignedInt(bytes[2], bytes[3]),
		ir: convertBytesToSignedInt(bytes[4], bytes[5]),		
	};
}

function Decoder(bytes, port) {
	if (port === 1) {
		return DecodeVoltage(bytes);
	} else if (port === 2 ) {
		return DecodeLSM9DS1Sensor(bytes);
	} else if (port === 8 ) {
		return DecodeSI1132Sensor(bytes);
	} else if (port === 0) {
		//nothing
		return {};
	}

	return {
		error: "unexpected port",
		port: port
	};
}