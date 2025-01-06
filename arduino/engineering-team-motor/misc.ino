unsigned long unsignedLongFromBytes(unsigned char* byte_ptr) {                                          
  unsigned long long_value = 0;                                        
  long_value =  ((unsigned long)*byte_ptr++ << 24); 
  long_value += ((unsigned long)*byte_ptr++ << 16); 
  long_value += ((unsigned long)*byte_ptr++ << 8);
  long_value +=  (unsigned long)*byte_ptr;  
  return long_value;
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
