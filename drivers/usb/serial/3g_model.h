// #######################################################
// # Option GlobeSurfer Icon (aka "Vodafone EasyBox")
// #
// # The message SHOULD be the same for all Option devices
	{ USB_DEVICE(0x0af0, 0x6600) },
// #######################################################
// # Option GlobeSurfer Icon 7.2 
// #
// # Contributor: The Pharscape Forum
	{ USB_DEVICE(0x0af0, 0x6901) },
// ########################################################
// # Option GlobeTrotter GT MAX 3.6 (aka "T-Mobile Web'n'walk Card Compact II")
// #
// # Contributor: Bernd Holzm�ller
	{ USB_DEVICE(0x0af0, 0x6600) },
// # ResponseEndpoint=0x84
// ########################################################
// # Option GlobeTrotter GT MAX "7.2 Ready"
// #
// # Contributors: Lucas Benedicic, Morgwai Kotarbinski
	{ USB_DEVICE(0x0af0, 0x6701) },
// ########################################################
// # Option GlobeTrotter EXPRESS 7.2 (aka "T-Mobile wnw Express II")
// #
// # Contributor: Fridtjof Busse
	{ USB_DEVICE(0x0af0, 0x6701) },
// # Response reading needed according to one (1) report
// #######################################################
// # Option GlobeSurfer Icon 7.2, new firmware (HSO driver)
// #
// # Vendor/ProductID don't change when switching, only the device
// # class does. Most new Option devices work with this. Just adapt IDs!
// # HSO driver support is available at Pharscape (www.pharscape.org)
// #
// # Contributor: Paul Hardwick
// ########################################################
// # Option iCON 210
// # PROLiNK PHS100 (various looks)
// # Hyundai Mobile MB-810
// #
// # One report of switching with DetachStorageOnly. Needs at least
// # a second to settle before binding to usbserial
// #
// # Contributor: wahlm, Peter Kraker, Pakdhetimin Sekum
	{ USB_DEVICE(0x1e0e, 0x9000) },
// #######################################################
// # Option iCON 225 HSDPA
// #
// # New Firmware. HSO driver support is available at Pharscape (www.pharscape.org)
// #
// # Contributor: Matti Viljanen
// #######################################################
// # Option GlobeTrotter HSUPA Modem (aka "T-Mobile Web'n'walk Card Compact III')
// #
// # New Firmware. HSO driver support is available at Pharscape (www.pharscape.org)
// #
// # Contributor: Gerold Gruber
// ###################################################### 
// # Option iCON 401
// #
// # HSO driver 
// #
// # Contributor: Vincent Teoh
// ######################################################## 
// # Vodafone K3760 (made by Option, HSO driver) 
// #
// # Contributor: The Solutor
// ########################################################
// # AT&T USBConnect Quicksilver (made by Option, HSO driver)
// #
// # Contributor: sissie from Bullteam
// ########################################################
// # Huawei devices
// #
// # Contributor: Hans Kurent, Denis Sutter, Vincent Teoh
// ########################################################
// # Huawei E169
// #
// # Contributor: Dale Lane
// # choose one of these:
// ########################################################
// # Huawei E180
// #
// # Contributor: Tom Dawahare
// ########################################################
// # Huawei E630
// #
// # There seem to be modem-only variants around - no storage,
// # no switching
// #
// # Contributor: Joakim Wenrgren
	{ USB_DEVICE(0x12d1, 0x1003) },
// ########################################################
// # ZTE MF620 (aka "Onda MH600HS")
// #
// # Probably works with DetachStorageOnly too
// #
// # Contributor: Flávio Moringa and others
	{ USB_DEVICE(0x19d2, 0x0001) },
// ########################################################
// # ZTE MF622 (aka "Onda MDC502HS"), MF100 and others
// #
// # Contributor: andylog
	{ USB_DEVICE(0x19d2, 0x0002) },
// ########################################################
// # ZTE MF628
// #
// # Captured with "usbmon". Has a micro SD slot which can be
// # activated alternatively
// #
// # Contributor: Alvaro Lopes <alvieboy at alvie dot com>
// # To modem mode:
	{ USB_DEVICE(0x19d2, 0x0015) },
// # To SD slot mode:
	{ USB_DEVICE(0x05c6, 0x2001) },
// ########################################################
// # ZTE MF622 (aka "Onda MDC502HS")
// # ZTE MF626
// # ZTE MF628+ (tested version from Telia / Sweden)
// # ZTE MF633
// # ZTE MF636 (aka "Telstra / BigPond 7.2 Mobile Card")
// # ZTE MF637
// #
// # Contributor: Joakim Wennergren and others
	{ USB_DEVICE(0x19d2, 0x0031) },
// ########################################################
// # ZTE MF638 (aka "Onda MDC525UP")
// #
// # Contributor: andylog
	{ USB_DEVICE(0x19d2, 0x0037) },
// ########################################################
// # ZTE AC8710
// # ZTE AC2726
// # and others
// # ZTE AC581 0xffeb //cym 20110829
// #
// # Many new ZTE devices use this sequence. There are
// # several ID combinations; check your default
// #
// # Contributor: Michael Khurtsiya, Amit Pundir and others
	{ USB_DEVICE(0x19d2, 0xffff) },
// # No. 2
	{ USB_DEVICE(0x19d2, 0xfff1) },
// # No. 3
	{ USB_DEVICE(0x19d2, 0xfff1) },
// # No. 4	
	{ USB_DEVICE(0x19d2, 0xffeb) }, 
// ########################################################
// # ZTE AC2710 (EVDO) 
// #
// # Contributor: Wasim Baig
	{ USB_DEVICE(0x19d2, 0xffff) },
// # Just for information: try it with the message from the AC8710 ...
// ########################################################
// # ZTE 6535-Z
// #
// # Contributor: David Taillandier
	{ USB_DEVICE(0x19d2, 0x0052) },
// ########################################################
// # ONDA MT503HS
// #
// # Contributor: Lucio Asnaghi a.k.a. kRAkEn/gORe
	{ USB_DEVICE(0x19d2, 0x0002) },
// ########################################################
// # ONDA MT505UP (most likely a ZTE model)
// #
// # Contributor: Alex Scortegagna
	{ USB_DEVICE(0x19d2, 0x0002) },
// ########################################################
// # Novatel Wireless Ovation MC950D HSUPA
// # Novatel Wireless Merlin XU950D
// # Novatel Wireless Ovation 930D
// #
// # Contributor: Razvan Dragomirescu, Mike Kirk
	{ USB_DEVICE(0x1410, 0x4400) },
// ########################################################
// # Novatel U727 USB modem
// #
// # Modem only versions (no switching required) are around.
// #
// # Contributor: Chris Thielen
	{ USB_DEVICE(0x1410, 0x4100) },
// ########################################################
// # Novatel MC990D
// #
// # Contributor: Joakim Wennergren
// ########################################################
// # Novatel U760 USB modem
// #
// # Contributor: Richard Laager
	{ USB_DEVICE(0x1410, 0x6000) },
// ########################################################
// # Alcatel One Touch X020 (aka OT-X020, aka MBD-100HU, aka Nuton 3.5G), works with Emobile D11LC
// # Alcatel One Touch X030 (aka OT-X030, aka Nuton NT36HD)
// #
// # Contributor: Aleksandar Samardzic, Marcelo Fernandez
	{ USB_DEVICE(0x1c9e, 0x6061) },
// ########################################################
// # Alcatel X200/X060S
	{ USB_DEVICE(0x1bbb, 0x0000) },
// ########################################################
// # AnyDATA ADU-500A, ADU-510A, ADU-510L, ADU-520A
// #
// # This ID gets an "Option" treatment by newer kernels.
// # Use module option "option_zero_cd=2" with usb-storage.
// # A kernel patch to fix the problem is pending
// #
// # Contributor: Vladimir Poluektov, Gabriel Smolar
	{ USB_DEVICE(0x16d5, 0x6502) },
// ########################################################
// # BandLuxe C120
// #
// # Reportedly needs several switch calls in a certain order. See
// # http://www.draisberghof.de/usb_modeswitch/bb/viewtopic.php?t=71
// # This might be a case to try the new ResetUSB function
// #
// # Contributor: Alexander Czigler
	{ USB_DEVICE(0x1a8d, 0x1002) },
// ########################################################
// # Solomon S3Gm-660
// #
// # Same notes apply as with the BandLuxe C120 above
// #
// # Contributor: Alexander Czigler
	{ USB_DEVICE(0x1dd6, 0x1002) },
// ########################################################
// # C-motech D-50 (aka "CDU-680")
// #
// # Interesting notes about the device in the forum post at
// # http://www.draisberghof.de/usb_modeswitch/bb/viewtopic.php?t=69
// #
// # Contributor: Alexander Czigler
	{ USB_DEVICE(0x16d8, 0x680a) },
// ########################################################
// # C-motech CGU-628 (aka "Franklin Wireless CGU-628A" aka "4G Systems XS Stick W12")
// #
// # Contributor: Mathias Picker
	{ USB_DEVICE(0x16d8, 0x6006) },
// ########################################################
// # Toshiba G450
// #
// # Contributor: Mijail Anton
	{ USB_DEVICE(0x0930, 0x0d45) },
// ########################################################
// # UTStarcom UM175 (distributor "Alltel")
// #
// # Contributor: Mark A. Ziesemer
	{ USB_DEVICE(0x106c, 0x3715) },
// ########################################################
// # Hummer DTM5731
// #
// # Contributor: "paul"
	{ USB_DEVICE(0x1ab7, 0x5731) },
// ########################################################
// # A-Link 3GU
// #
// # Contributor: A-Link Europe Ltd.
	{ USB_DEVICE(0x1e0e, 0x9200) },
// ########################################################
// # Sierra Wireless Compass 597
// #
// # Contributor: Vincent Teoh
	{ USB_DEVICE(0x1199, 0x0023) },
// ########################################################
// # Sierra Wireless AirCard 881U (most likely 880U too)
// #
// # Contributor: Vincent Teoh
	{ USB_DEVICE(0x1199, 0x6856) },
// ########################################################
// # Sony Ericsson MD400
// #
// # Special procedure, takes around 25 secs. on the whole
// ########################################################
// # LG LDU-1900D EV-DO (Rev. A)
// #
// # Recommended init command: ATE0V1&D2&C1S0=0
// #
// # Contributor: Jérôme Oufella
// ########################################################
// # Samsung SGH-Z810 USB (with microSD card)
// #
// # This ID gets a wrong "Option" treatment by newer kernels
// # from 2.6.29 to 2.6.31
// # Use module option "option_zero_cd=2" with usb-storage.
// # Problem fixed in 2.6.32
// #
// # Contributor: A Friend
	{ USB_DEVICE(0x04e8, 0x6601) },
// ######################################################## 
// # MobiData MBD-200HU
// #
// # Contributor: Stefan Olejnik
	{ USB_DEVICE(0x1c9e, 0x9000) },
// ########################################################
// # ST Mobile Connect HSUPA USB Modem
// #
// # Use /dev/ttyUSB2 for connecting
// #
// # Contributor: Vincent Teoh
	{ USB_DEVICE(0x1c9e, 0x9063) },
// ########################################################
// # MyWave SW006 Sport Phone/Modem Combination
// #
// # Has a card reader too, working fine
// # 
// # Contributor: Aaron Guidazzi
	{ USB_DEVICE(0x1c9e, 0x9202) },
// ########################################################
// # Cricket A600
// #
// # Switches to ACM device. Might need a ResetUSB after switching - or not
// #
// # Contributor: Jeffrey Hoogland
	{ USB_DEVICE(0x1f28, 0x0020) },
// # Afterwards (optional): "usb_modeswitch -v 0x1f28 -p 0x0020 -R"
// ########################################################
// # EpiValley SEC-7089 (featured by Alegro and Starcomms / iZAP)
// #
// # Contributor: Chris Wright
	{ USB_DEVICE(0x1b7d, 0x0001) },
// ########################################################
// # Samsung U209
// #
// # Has a modem and a storage device after switching
// #
// # Contributor: Arif Ahmed
	{ USB_DEVICE(0x04e8, 0x6601) },
// ########################################################
// # Huawei E270+  (HSPA+ modem)
// # Huawei E1762
// # Huawei E1820
// #
// # Contributor: Paranoid Paranoia
	{ USB_DEVICE(0x12d1, 0x14ac) },
// ########################################################
// # Huawei E1550
// # Huawei E1750
// #
// # Contributor: Anders Blomdell, Ahmed Soliman
	{ USB_DEVICE(0x12d1, 0x1001) },
// ########################################################
// # ZTE K3520-Z
// #
// # Contributor: Paul McDermott
	{ USB_DEVICE(0x19d2, 0x0055) },
// ######################################################## 
// # MobiData MBD-200HU (aka 4G XS Stick W10/W14, aka Micromax MMX 300G,
// # aka ChinaBird CBCPL68)
// #
// # Contributor: Chris
	{ USB_DEVICE(0x1c9e, 0x9603) },
// ######################################################## 
// # D-Link DWM-162-U5, Micromax MMX 300c
// #
// # Contributor: Zhang Le
	{ USB_DEVICE(0x1e0e, 0xce16) },
        { USB_DEVICE(0x1e0e, 0xcefe) },
// ######################################################## 
// # Novatel MC760 3G
// #
// # Contributor: Matt Roberds
	{ USB_DEVICE(0x1410, 0x6002) },
// ########################################################
// # ZTE MF110 (Variant)
// #
// # Contributor: Moritz Grosse-Wentrup
	{ USB_DEVICE(0x19d2, 0x0031) },
// ########################################################
// # Philips TalkTalk (NXP Semiconductors "Dragonfly")
// #
	{ USB_DEVICE(0x0471, 0x1234) },
// ########################################################
// # HuaXing E600 (NXP Semiconductors "Dragonfly")
// #
// # Contributor: Emfox Zhou
	{ USB_DEVICE(0x0471, 0x1206) },
// ########################################################
// # ZTE K3565
// #
	{ USB_DEVICE(0x19d2, 0x0063) },
// ########################################################
// # Motorola 802.11 bg WLAN (TER/GUSB3-E)
// #
// # Contributor: Prashad Prashant
	{ USB_DEVICE(0x148f, 0x9021) },
// ########################################################
// # Huawei E1612
// #
// # Contributor: Genar Codina
	{ USB_DEVICE(0x12d1, 0x1406) },
// ########################################################
// # Huawei E1690
// # Huawei E1692
// # Huawei E1762
// #
// # Contributor: Carolin Latze and others
	{ USB_DEVICE(0x12d1, 0x140c) },
// ########################################################
// # C-motech CHU-629S
// #
// ########################################################
// # Sagem F@ST 9520-35-GLR
// #
// # Contributor: Yaroslav Levandovskiy
	{ USB_DEVICE(0x1076, 0x7f00) },
// ########################################################
// # Nokia CS-15
// #
// # Contributor: Antti Turunen
	{ USB_DEVICE(0x0421, 0x0612) },
// ########################################################
// # Huawei K3765
// #
// # Contributor: Felix Schwarz
	{ USB_DEVICE(0x12d1, 0x1465) },
// ########################################################
// # Huawei K4505
// #
// # Contributor: Nikolaos Koutsianas
	{ USB_DEVICE(0x12d1, 0x1464) },
// #########################################
// # Vodafone MD950 (Wisue Technology)
// #
// # Contributor: Bob Williams
// # The report said that giving the target IDs prevented
// # the device from switching. Got to try it out ...
// #TargetVendor=   0x1dbc
// #TargetProduct=  0x0005
// ########################################################
// # Siptune LM-75 ("LinuxModem")
// #
// # Contributor: Antti Turunen
	{ USB_DEVICE(0x05c6, 0x9000) },
// ########################################################
// # Zydas ZD1211RW WLAN USB, Sphairon HomeLink 1202 (Variant 1)
// #
// # WLAN devices from Linux kernel
// ########################################################
// # Zydas ZD1211RW WLAN USB, Sphairon HomeLink 1202 (Variant 2)
// #
// # WLAN devices from Linux kernel
// ########################################################
// # Vertex Wireless 100 Series
// #
// # Contributor: Reinis Danne
	{ USB_DEVICE(0x1fe7, 0x0100) },
// ########################################################
// # AVM Fritz!Wlan USB Stick N
// #
	{ USB_DEVICE(0x057c, 0x8401) },
// ########################################################
// # InfoCert Business Key (SmartCard/Reader emulation)
// #
// # Contributor: Bicio
	{ USB_DEVICE(0x072f, 0x90cc) },
// ########################################################
// # UTStarcom UM185E (distributor "Alltel")
// #
// # Contributor: Jason Smith
	{ USB_DEVICE(0x106c, 0x3717) },
// ########################################################
// # ZTE AC581
// #
// # Contributor: Alberto Maurizi
	{ USB_DEVICE(0x19d2, 0x0094) },
// ########################################################
// # Huawei U7510 / U7517
// #
// # Contributor: Isaac Salgado
// ########################################################
// # Beceem BCSM250
// #
// # Contributor: Alexander Gordeev
	{ USB_DEVICE(0x198f, 0x0220) },
// ########################################################
// # LG HDM-2100 (EVDO Rev.A USB modem)
// #
// # Contributor: Jérôme Oufella
	{ USB_DEVICE(0x1004, 0x6114) },
// ########################################################
// # Kyocera W06K CDMA modem
// #
// # Contributor: Ying-Hung Lo
// ########################################################
// # Digicom 8E4455
// #
// # Contributor: Alberto Ciampini
	{ USB_DEVICE(0x1266, 0x1009) },
// ########################################################
// # Sony Ericsson MD300
// #
// # Contributor: Betavine Project
// ########################################################
// # Vodafone (ZTE) K3805-Z
// #
// # Contributor: Betavine Project
// #
// # Note:
// #   This device has multiple USB profiles. Depending upon how it is flipped
// #   from storage mode to modem mode determines its final PID and the packages
// #   shown on its ISO CD image.
// ########################################################
// # Franklin Wireless U210
// #
// # Contributor: Adam J. Porter
	{ USB_DEVICE(0x1fac, 0x0131) },
// ########################################################
// # Alcatel X220L, X215S
// #
// # Contributor: John Watt
	{ USB_DEVICE(0x1bbb, 0x0017) },
// ########################################################
// # ZTE MF112
// #
// # Contributors: Akos Ladanyi, John Talbut
	{ USB_DEVICE(0x19d2, 0x0031) },
// ########################################################
// # Olivetti Olicard 100 and others
// #
// # Contributor: Nils Radtke
	{ USB_DEVICE(0x0b3c, 0xc000) },
	{ USB_DEVICE(0x0b3c, 0xc001) },
	{ USB_DEVICE(0x0b3c, 0xc002) },

// ########################################################
// # ZTE MF110 (Variant)
// #
// # Contributor: uusrlnx
	{ USB_DEVICE(0x19d2, 0x0124) },
// ########################################################
// # ZTE MU351
// ########################################################
// # LG L-05A
	{ USB_DEVICE(0x1004, 0x6124) },
// ########################################################
// # LG LUU-2100TI (aka AT&T USBConnect Turbo)
// #
// # Contributor: Chris Jager
	{ USB_DEVICE(0x1004, 0x6141) },
// ######################################################## 
// # LG KP500 Cookie Phone  - UNTESTED! Please report!
// #
// # Contributor: Oliver Mattos
	{ USB_DEVICE(0x1004, 0x6000) },
// ########################################################
// # Royaltek Q110 - UNCONFIRMED!
	{ USB_DEVICE(0x1266, 0x1009) },
// ########################################################
// # Rogers Rocket Stick (a Sony Ericsson device)
// ########################################################
// # BandRich BandLuxe C170, BandLuxe C270
	{ USB_DEVICE(0x1a8d, 0x1009) },
// ########################################################
// # Vodafone (Huawei) K4605
// #
// # Contributor: Betavine Project
// ########################################################
// # Huawei R201
// #
// # Contributor: Betavine Project
// ########################################################
// # Atheros Wireless / Netgear WNDA3200
// #
// # Contributor: Sujith Manoharan (Atheros)
// ########################################################
// # Onda MW833UP
// #
// # Contributor: Riccardo Sepe aka zeroidle
// ########################################################
// # Huawei U8110 (Android smartphone)
// #
// # Contributor: David Erosa García
// # for Android SDK
// ########################################################
// # Nokia CS-10
// #
// # Contributor: Wacław Sierek
// ########################################################
// # ZTE WCDMA Stick from BNSL
// #
// # Contributor: Anoop John
	{ USB_DEVICE(0x19d2, 0x0108) },
// ########################################################
// # BSNL Capitel
// #######################################################
// # Samsung GT-B3730
// #
// # Contributor: Per Øyvind Karlsen
	{ USB_DEVICE(0x04e8, 0x6889) },
// ########################################################
// # Onda MW833UP
// #
// # Contributor: Enrico Mioso
	{ USB_DEVICE(0x1ee8, 0x000b) },
// ########################################################
// # Olivetti Olicard 145
// #
// # Contributor: Pierre-Andre Cornillon
	{ USB_DEVICE(0x0b3c, 0xc003) },
// #######################################################
// # Huawei EC168C (from Zantel)
// #
// # Contributor: Åsmund Hjulstad
	{ USB_DEVICE(0x12d1, 0x1412) },
