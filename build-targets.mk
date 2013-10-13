# SnoreCopter project target
$(BUILD_PATH)/SnoreCopter.o: SnoreCopter.cpp
	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBMAPLE_INCLUDES) $(WIRISH_INCLUDES) -o $@ -c $< 

Libraries/AQ_Math/AQMath.o: Libraries/AQ_Math/AQMath.cpp
	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBMAPLE_INCLUDES) $(WIRISH_INCLUDES) -o $@ -c $< 

#SoftwareSerial_NB.o: SoftwareSerial_NB.cpp
#	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBMAPLE_INCLUDES) $(WIRISH_INCLUDES) -o $@ -c $< 

build/MapleCompatibility/flash_stm32.o: MapleCompatibility/flash_stm32.c
	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBMAPLE_INCLUDES) $(WIRISH_INCLUDES) -o $@ -c $< 

#build/AeroQuad/AeroQuad.o: AeroQuad/AeroQuad.cpp
#	$(SILENT_CXX) $(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBMAPLE_INCLUDES) $(WIRISH_INCLUDES) -o $@ -c $< 



$(BUILD_PATH)/libmaple.a: $(BUILDDIRS) $(TGT_BIN)
	- rm -f $@
	$(AR) crv $(BUILD_PATH)/libmaple.a $(TGT_BIN)

library: $(BUILD_PATH)/libmaple.a

.PHONY: library

$(BUILD_PATH)/$(BOARD).elf: $(BUILDDIRS) $(TGT_BIN) $(BUILD_PATH)/SnoreCopter.o
	$(SILENT_LD) $(CXX) $(LDFLAGS) -o $@ $(TGT_BIN) $(BUILD_PATH)/SnoreCopter.o -Wl,-Map,$(BUILD_PATH)/$(BOARD).map

$(BUILD_PATH)/$(BOARD).bin: $(BUILD_PATH)/$(BOARD).elf
	$(SILENT_OBJCOPY) $(OBJCOPY) -v -Obinary $(BUILD_PATH)/$(BOARD).elf $@ 1>/dev/null
	$(SILENT_DISAS) $(DISAS) -d $(BUILD_PATH)/$(BOARD).elf > $(BUILD_PATH)/$(BOARD).disas
	@echo " "
	@echo "Object file sizes:"
	@find $(BUILD_PATH) -iname *.o | xargs $(SIZE) -t > $(BUILD_PATH)/$(BOARD).sizes
	@cat $(BUILD_PATH)/$(BOARD).sizes
	@echo " "
	@echo "Final Size:"
	@$(SIZE) $<
	@echo $(MEMORY_TARGET) > $(BUILD_PATH)/build-type
	- rm $(BUILD_PATH)/SnoreCopter.o

$(BUILDDIRS):
	@mkdir -p $@

MSG_INFO:
	@echo "================================================================================"
	@echo ""
	@echo "  Build info:"
	@echo "     BOARD:          " $(BOARD)
	@echo "     MCU:            " $(MCU)
	@echo "     MEMORY_TARGET:  " $(MEMORY_TARGET)
	@echo ""
	@echo "  See 'make help' for all possible targets"
	@echo ""
	@echo "================================================================================"
	@echo ""
