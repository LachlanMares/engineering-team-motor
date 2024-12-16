unsigned long unsignedLongFromBytes(unsigned char* byte_ptr) {                                          
  unsigned long long_value = 0;                                        
  long_value =  ((unsigned long)*byte_ptr++ << 24); 
  long_value += ((unsigned long)*byte_ptr++ << 16); 
  long_value += ((unsigned long)*byte_ptr++ << 8);
  long_value +=  (unsigned long)*byte_ptr;  
  return long_value;
}

void bytesFromUnsignedLong(unsigned long long_value, unsigned char* byte_ptr) {                                          
  *byte_ptr++ = long_value >> 24;                                      
  *byte_ptr++ = long_value >> 16;   
  *byte_ptr++ = long_value >> 8;   
  *byte_ptr = long_value;  
}

void bytesFromFloat16Bit(float float_value, unsigned char* byte_ptr, uint8_t multiplier) {
  int16_t float_value_int16 = (int16_t)(float_value * (float)(multiplier));
  *byte_ptr++ = float_value_int16 >> 8;
  *byte_ptr++ = float_value_int16;
  *byte_ptr = multiplier;
}

void bytesFromFloat32Bit(float float_value, unsigned char* byte_ptr, uint16_t multiplier) {
  int32_t float_value_int32 = (int32_t)(float_value * (float)(multiplier));
  *byte_ptr++ = float_value_int32 >> 24;                                      
  *byte_ptr++ = float_value_int32 >> 16;   
  *byte_ptr++ = float_value_int32 >> 8;   
  *byte_ptr++ = float_value_int32;  
  *byte_ptr++ = multiplier >> 8;   
  *byte_ptr = multiplier;  
}

long longFromBytes(unsigned char* byte_ptr) {                                          
  long long_value = 0;                                        
  long_value =  ((long)*byte_ptr++ << 24); 
  long_value += ((long)*byte_ptr++ << 16); 
  long_value += ((long)*byte_ptr++ << 8);
  long_value +=  (long)*byte_ptr;  
  return long_value;
}

int intFromBytes(unsigned char* byte_ptr) {                                          
  int int_value = 0;                                        
  int_value = ((int)*byte_ptr++ << 8);
  int_value += (int)*byte_ptr;  
  return int_value;
}

void bytesFromLong(long long_value, unsigned char* byte_ptr) {                                          
  *byte_ptr++ = long_value >> 24;                                      
  *byte_ptr++ = long_value >> 16;   
  *byte_ptr++ = long_value >> 8;   
  *byte_ptr = long_value;  
}

void bytesFromInt(int int_value, unsigned char* byte_ptr) {                                          
  *byte_ptr++ = int_value >> 8;   
  *byte_ptr = int_value;  
}
