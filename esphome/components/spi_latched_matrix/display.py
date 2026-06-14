from esphome import pins
import esphome.codegen as cg
from esphome.components import display, spi
import esphome.config_validation as cv
from esphome.const import (
    CONF_ENABLE_PIN,
    CONF_HEIGHT,
    CONF_ID,
    CONF_LAMBDA,
    CONF_MAX_BRIGHTNESS,
    CONF_PAGES,
    CONF_PIXEL_MAPPER,
    CONF_THRESHOLD,
    CONF_WIDTH,
)

DEPENDENCIES = ["spi"]

CONF_LATCH_PIN = "latch_pin"
CONF_INVERT_ENABLE = "invert_enable"
CONF_GRAY_LEVELS = "gray_levels"
CONF_REFRESH_INTERVAL = "refresh_interval"

spi_latched_matrix_ns = cg.esphome_ns.namespace("spi_latched_matrix")
SPILatchedMatrix = spi_latched_matrix_ns.class_(
    "SPILatchedMatrix", display.DisplayBuffer, spi.SPIDevice
)


CONFIG_SCHEMA = cv.All(
    display.FULL_DISPLAY_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(SPILatchedMatrix),
            cv.Required(CONF_WIDTH): cv.positive_int,
            cv.Required(CONF_HEIGHT): cv.positive_int,
            cv.Required(CONF_LATCH_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_INVERT_ENABLE, default=False): cv.boolean,
            cv.Optional(CONF_THRESHOLD, default=1): cv.int_range(min=1, max=255),
            cv.Optional(CONF_MAX_BRIGHTNESS, default="100%"): cv.percentage,
            cv.Optional(CONF_GRAY_LEVELS, default=1): cv.int_range(min=1, max=255),
            cv.Optional(
                CONF_REFRESH_INTERVAL, default="200us"
            ): cv.positive_time_period_microseconds,
            cv.Optional(CONF_PIXEL_MAPPER): cv.returning_lambda,
        }
    ).extend(spi.spi_device_schema(False, "10MHz")),
    cv.has_at_most_one_key(CONF_PAGES, CONF_LAMBDA),
)

FINAL_VALIDATE_SCHEMA = spi.final_validate_device_schema(
    "spi_latched_matrix", require_miso=False, require_mosi=True
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_WIDTH], config[CONF_HEIGHT])
    await display.register_display(var, config)
    await spi.register_spi_device(var, config, write_only=True)

    latch_pin = await cg.gpio_pin_expression(config[CONF_LATCH_PIN])
    cg.add(var.set_latch_pin(latch_pin))

    if enable_pin_config := config.get(CONF_ENABLE_PIN):
        enable_pin = await cg.gpio_pin_expression(enable_pin_config)
        cg.add(var.set_enable_pin(enable_pin))

    cg.add(var.set_invert_enable(config[CONF_INVERT_ENABLE]))
    cg.add(var.set_threshold(config[CONF_THRESHOLD]))
    cg.add(var.set_max_brightness(round(config[CONF_MAX_BRIGHTNESS] * 255)))
    cg.add(var.set_gray_levels(config[CONF_GRAY_LEVELS]))
    cg.add(
        var.set_refresh_interval_us(config[CONF_REFRESH_INTERVAL].total_microseconds)
    )

    if pixel_mapper := config.get(CONF_PIXEL_MAPPER):
        pixel_mapper_template = await cg.process_lambda(
            pixel_mapper,
            [(int, "x"), (int, "y")],
            return_type=cg.int_,
        )
        cg.add(var.set_pixel_mapper(pixel_mapper_template))

    if lambda_config := config.get(CONF_LAMBDA):
        lambda_ = await cg.process_lambda(
            lambda_config, [(display.DisplayRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))

    display.add_metadata(
        config[CONF_ID],
        width=config[CONF_WIDTH],
        height=config[CONF_HEIGHT],
        has_writer=CONF_LAMBDA in config or CONF_PAGES in config,
    )
