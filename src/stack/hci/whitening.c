#include "string.h"
#include "whitening.h"

void little_endian_store_8(uint8_t *buffer, uint16_t *pos, uint8_t value){
    buffer[(*pos)++] = (uint8_t)(value);
}

void little_endian_store_16(uint8_t *buffer, uint16_t *pos, uint16_t value){
    buffer[(*pos)++] = (uint8_t)(value);
    buffer[(*pos)++] = (uint8_t)(value >> 8);
}

void little_endian_store_24(uint8_t *buffer, uint16_t *pos, uint32_t value){
    buffer[(*pos)++] = (uint8_t)(value);
    buffer[(*pos)++] = (uint8_t)(value >> 8);
    buffer[(*pos)++] = (uint8_t)(value >> 16);
}

void little_endian_store_32(uint8_t *buffer, uint16_t *pos, uint32_t value){
    buffer[(*pos)++] = (uint8_t)(value);
    buffer[(*pos)++] = (uint8_t)(value >> 8);
    buffer[(*pos)++] = (uint8_t)(value >> 16);
    buffer[(*pos)++] = (uint8_t)(value >> 24);
}

void little_endian_store_string(uint8_t *buffer, uint16_t *pos, const uint8_t *value, uint16_t length){
    memcpy(&buffer[*pos], value, length);
    *pos += length;
}

void big_endian_store_string(uint8_t *buffer, uint16_t *pos, const uint8_t *value, uint16_t length){
    for (uint16_t i = 0; i < length; i++) {
        buffer[(*pos)++] = value[length-i-1];
    }
}

uint8_t xn297_invert_8bit ( uint8_t data )
{
	uint8_t temp = 0;

	for ( uint8_t i = 0; i < 8; i++ )
	{
		if (data & (1 << i))
		{
			temp |= 1 << (7 - i);
		}
	}
    
	return temp;
}

uint16_t xn297_invert_16bit ( uint16_t data )
{
	uint16_t temp = 0;

	for ( uint8_t i = 0; i < 16; i++ )
	{
		if (data & (1 << i))
		{
			temp |= 1 << (15 - i);
		}
	}

	return temp;
}

uint16_t xn297_crc16 ( const uint8_t* address, 
                       uint8_t        address_length, 
                       const uint8_t* payload, 
                       uint8_t        payload_length )
{
	uint16_t crc  = 0xFFFF;
	uint16_t poly = 0x1021;
	uint8_t  temp = 0;

	for ( uint8_t i = 0; i < address_length; i++ )
	{
		// Addr: invert endian
		temp = address[address_length - 1 - i];

		crc ^= (temp << 8);

		for (int j = 0; j < 8; j++)
		{
			if (crc & 0x8000)
			{
				crc = (crc << 1) ^ poly;
			}
			else
			{
				crc = (crc << 1);
			}
		}
	}

	for ( uint8_t i = 0; i < payload_length; i++ )
	{
		// Payload: invert bit order
		temp = xn297_invert_8bit ( payload[i] );

		crc ^= (temp << 8);

		for (int j = 0; j < 8; j++)
		{
			if (crc & 0x8000)
			{
				crc = (crc << 1) ^ poly;
			}
			else
			{
				crc = (crc << 1);
			}
		}
	}

	crc = xn297_invert_16bit ( crc );

	return (crc ^ 0xFFFF);
}

void xn297_whitening_init ( uint8_t rf_channel, uint32_t *reg )
{
	reg[0] = 1;
	
	for ( uint8_t i = 1; i < 7; i++ )
	{
		reg[i] = (rf_channel >> (6 - i)) & 0x01;
	}
}

uint32_t xn297_whitening_output ( uint32_t *reg )
{
	uint32_t temp = reg[3] ^ reg[6];
	
	reg[3] = reg[2];
	reg[2] = reg[1];
	reg[1] = reg[0];
	reg[0] = reg[6];
	reg[6] = reg[5];
	reg[5] = reg[4];
	reg[4] = temp;
	
	return reg[0];
}

void xn297_whitenging_encode ( uint8_t *data, uint8_t length, uint32_t *init_reg, uint8_t ignore_length )
{    
    for ( uint8_t i = 0; i < ignore_length; i++ )
	{
		int data_input = data[i];
		int data_bit = 0;
		int data_output = 0;
		
		for ( uint8_t bit_index = 0; bit_index < 8; bit_index++ )
		{
			data_bit = (data_input >> (bit_index)) & 0x01;
			
			data_bit ^= xn297_whitening_output ( init_reg );
			
			data_output += (data_bit << (bit_index));
		}
	}
    
	for ( uint8_t i = 0; i < length; i++ )
	{
		int data_input = data[i];
		int data_bit = 0;
		int data_output = 0;
		
		for (int bit_index = 0; bit_index < 8; bit_index++)
		{
			data_bit = (data_input >> (bit_index)) & 0x01;
			
			data_bit ^= xn297_whitening_output ( init_reg );
			
			data_output += (data_bit << (bit_index));
		}
		
		data[i] = data_output;
	}
}

uint8_t xn297_whitening_payload_generate ( uint8_t        rf_channel, 
                                           const uint8_t* address, 
                                           uint8_t        address_length, 
                                           const uint8_t* payload, 
                                           uint8_t        payload_length, 
                                           uint8_t*       dest )
{
    uint8_t  length = 0;
    uint32_t reg_init_ble[7]   = { 0 };
	uint32_t reg_init_xn297[7] = { 0 };
    
    xn297_whitening_init ( rf_channel, reg_init_ble );
	xn297_whitening_init ( 0x3F, reg_init_xn297 );
    
    // premable 3 Bytes, address 3/5 Bytes, payload , crc16 3 Bytes
    if ( ( address_length + payload_length + XN297_PREMABLE_SIZE + XN297_CRC_SIZE ) > BLE_PDU_PAYLOAD_SIZE )
    {
        return length;
    }
    
    uint16_t pos = 0;
    
    /*** Step1. copy pre, address and rf payload ***/
    uint32_t xn297_premable = 0x550F71;
    // premable
    little_endian_store_24 ( dest, &pos, xn297_premable );
    // address
    big_endian_store_string ( dest, &pos, address, address_length );
    // payload
    little_endian_store_string ( dest, &pos, payload, payload_length );
    
    /*** Step2. XN297L bit invert premable & address ***/
	for ( uint8_t i = 0; i < XN297_PREMABLE_SIZE + address_length; i++)
	{
		dest[i] = xn297_invert_8bit ( dest[i] );
	}
    
	/*** Step3. add crc16 ***/
	uint16_t crc = xn297_crc16 ( address, address_length, payload, payload_length );
    little_endian_store_16 ( dest, &pos, crc );
    
	/*** Step4. xn297l whitening ***/
	xn297_whitenging_encode ( dest + XN297_PREMABLE_SIZE, 
                              address_length + payload_length + XN297_CRC_SIZE, 
                              reg_init_xn297, 
                              0 );
    
	/*** Step5. BLE whitening ***/
//	xn297_whitenging_encode ( dest, 
//                              XN297_PREMABLE_SIZE + address_length + payload_length + XN297_CRC_SIZE, 
//                              reg_init_ble, 
//                              BLE_PDU_HEADER_SIZE );
//    
    return pos;
}

