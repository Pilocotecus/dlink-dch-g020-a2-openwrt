#!/bin/sh

echo "[BA HOME] Activando servicios al arranque"

/etc/init.d/uhttpd enable
/etc/init.d/bernal-zwave enable

/etc/init.d/uhttpd restart
/etc/init.d/bernal-zwave restart

echo "[BA HOME] Servicios activados"
