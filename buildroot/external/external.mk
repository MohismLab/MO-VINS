include $(sort $(wildcard $(BR2_EXTERNAL_MOHISM_PATH)/package/*/*.mk))

# Vision throughput matters more than size for these libraries. Preserve IEEE
# NaN/Inf semantics used by the estimator's rejection gates.
OPENCV4_CXXFLAGS += -O3
OPENCV4_CONF_OPTS += -DENABLE_FAST_MATH=OFF
