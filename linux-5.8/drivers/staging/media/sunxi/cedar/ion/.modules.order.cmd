cmd_drivers/staging/media/sunxi/cedar/ion/modules.order := {   cat drivers/staging/media/sunxi/cedar/ion/sunxi/modules.order; :; } | awk '!x[$$0]++' - > drivers/staging/media/sunxi/cedar/ion/modules.order
