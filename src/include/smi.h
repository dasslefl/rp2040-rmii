
// smi.h

#ifndef __SMI_H__
#define __SMI_H__

#define PHY_REG_BCR 0
#define PHY_REG_BCR_SOFT_RESET (1 << 15)
#define PHY_REG_BCR_LOOPBACK (1 << 14)
#define PHY_REG_BCR_SPEED_100 (1 << 13)
#define PHY_REG_BCR_AUTO_NEGOTIATION_ENABLE (1 << 12)
#define PHY_REG_BCR_FULL_DUPLEX (1 << 8)

#define PHY_REG_BSR 1
#define PHY_REG_BSR_AUTO_NEGOTIATION_COMPLETE (1 << 5)
#define PHY_REG_BSR_LINK_UP (1 << 2)

#define PHY_REG_PIR1 2
#define PHY_REG_PIR1_DEFAULT_VALUE 0x0007

#define PHY_REG_SECR 26

void smi_init();

uint16_t smi_reg_read(uint8_t phy_addr, uint8_t reg_addr);
uint16_t smi_reg_read_bits(uint8_t phy_addr, uint8_t reg_addr, uint16_t bits);

void smi_reg_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data);
void smi_reg_set_bits(uint8_t phy_addr, uint8_t reg_addr, uint16_t bits);

#endif // __SMI_H__
