def build(bld):
    module = bld.create_ns3_module('ladzrp', ['internet'])
    module.includes = '.'
    module.source = [
        'model/ladzrp-rtable.cc',
        'model/ladzrp-packet.cc',
        'model/ladzrp-routing-protocol.cc',
        'model/ladzrp-id-cache.cc',
        'model/ladzrp-rqueue.cc',
        'helper/ladzrp-helper.cc'
        ]

    headers = bld(features='ns3header')
    headers.module = 'ladzrp'
    headers.source = [
        'model/ladzrp-rtable.h',
        'model/ladzrp-packet.h',
        'model/ladzrp-routing-protocol.h',
        'model/ladzrp-id-cache.h',
        'model/ladzrp-rqueue.h',
        'helper/ladzrp-helper.h',
        ]
    if (bld.env['ENABLE_EXAMPLES']):
      bld.recurse('examples')

    bld.ns3_python_bindings()