"""Numba JIT kernel infrastructure for accelerated dynamics simulation.

This module provides:
- JIT function registry mapping model names to their compiled functions
- prepare_jit_data(): packs metadata dicts into flat arrays and resolves JIT functions
- warmup_jit(): triggers compilation of all JIT functions before simulation
"""
import numpy as np

# Import JIT functions and packers from each component
from components.generators.genrou import (
    genrou_dynamics_jit, pack_genrou_meta,
    GENROU_PORTS_SIZE, GENROU_META_SIZE
)
from components.exciters.exdc2 import (
    exdc2_dynamics_jit, exdc2_output_jit, pack_exdc2_meta,
    EXDC2_PORTS_SIZE, EXDC2_META_SIZE
)
from components.exciters.exst1 import (
    exst1_dynamics_jit, exst1_output_jit, pack_exst1_meta,
    EXST1_PORTS_SIZE, EXST1_META_SIZE
)
from components.exciters.esst3a import (
    esst3a_dynamics_jit, esst3a_output_jit, pack_esst3a_meta,
    ESST3A_PORTS_SIZE, ESST3A_META_SIZE
)
from components.exciters.ieeex1 import (
    ieeex1_dynamics_jit, ieeex1_output_jit, pack_ieeex1_meta,
    IEEEX1_PORTS_SIZE, IEEEX1_META_SIZE
)
from components.governors.tgov1 import (
    tgov1_dynamics_jit, tgov1_output_jit, pack_tgov1_meta,
    TGOV1_PORTS_SIZE, TGOV1_META_SIZE
)
from components.governors.ieeeg1 import (
    ieeeg1_dynamics_jit, ieeeg1_output_jit, pack_ieeeg1_meta,
    IEEEG1_PORTS_SIZE, IEEEG1_META_SIZE
)

# Registry: model_name -> (dynamics_jit, output_jit, pack_meta, ports_size, meta_size)
JIT_REGISTRY = {
    # Generators (no output_fn - outputs computed in network solution)
    'GENROU': {
        'dynamics_jit': genrou_dynamics_jit,
        'output_jit': None,
        'pack_meta': pack_genrou_meta,
        'ports_size': GENROU_PORTS_SIZE,
        'meta_size': GENROU_META_SIZE,
    },
    # Exciters
    'EXDC2': {
        'dynamics_jit': exdc2_dynamics_jit,
        'output_jit': exdc2_output_jit,
        'pack_meta': pack_exdc2_meta,
        'ports_size': EXDC2_PORTS_SIZE,
        'meta_size': EXDC2_META_SIZE,
    },
    'EXST1': {
        'dynamics_jit': exst1_dynamics_jit,
        'output_jit': exst1_output_jit,
        'pack_meta': pack_exst1_meta,
        'ports_size': EXST1_PORTS_SIZE,
        'meta_size': EXST1_META_SIZE,
    },
    'ESST3A': {
        'dynamics_jit': esst3a_dynamics_jit,
        'output_jit': esst3a_output_jit,
        'pack_meta': pack_esst3a_meta,
        'ports_size': ESST3A_PORTS_SIZE,
        'meta_size': ESST3A_META_SIZE,
    },
    'IEEEX1': {
        'dynamics_jit': ieeex1_dynamics_jit,
        'output_jit': ieeex1_output_jit,
        'pack_meta': pack_ieeex1_meta,
        'ports_size': IEEEX1_PORTS_SIZE,
        'meta_size': IEEEX1_META_SIZE,
    },
    # Governors
    'TGOV1': {
        'dynamics_jit': tgov1_dynamics_jit,
        'output_jit': tgov1_output_jit,
        'pack_meta': pack_tgov1_meta,
        'ports_size': TGOV1_PORTS_SIZE,
        'meta_size': TGOV1_META_SIZE,
    },
    'IEEEG1': {
        'dynamics_jit': ieeeg1_dynamics_jit,
        'output_jit': ieeeg1_output_jit,
        'pack_meta': pack_ieeeg1_meta,
        'ports_size': IEEEG1_PORTS_SIZE,
        'meta_size': IEEEG1_META_SIZE,
    },
}


def prepare_jit_data(builder):
    """
    Pack all component metadata into flat arrays and resolve JIT functions.

    Called once after system build and initialization (after power flow sets
    final metadata values like Vref, Pref, etc.).

    Args:
        builder: PowerSystemBuilder with built components

    Returns:
        dict with per-machine JIT data:
            gen_meta_arr: list of packed gen meta arrays
            exc_meta_arr: list of packed exc meta arrays
            gov_meta_arr: list of packed gov meta arrays
            gen_dyn_jit: list of JIT dynamics functions per gen
            exc_dyn_jit: list of JIT dynamics functions per exc
            exc_out_jit: list of JIT output functions per exc
            gov_dyn_jit: list of JIT dynamics functions per gov
            gov_out_jit: list of JIT output functions per gov
            gen_ports_buf: list of pre-allocated port buffers
            exc_ports_buf: list of pre-allocated port buffers
            gov_ports_buf: list of pre-allocated port buffers
    """
    n_gen = len(builder.generators)

    jit_data = {
        'gen_meta_arr': [],
        'exc_meta_arr': [],
        'gov_meta_arr': [],
        'gen_dyn_jit': [],
        'exc_dyn_jit': [],
        'exc_out_jit': [],
        'gov_dyn_jit': [],
        'gov_out_jit': [],
        'gen_ports_buf': [],
        'exc_ports_buf': [],
        'gov_ports_buf': [],
    }

    for i in range(n_gen):
        gen_core = builder.generators[i]
        exc_core = builder.exciters[i]
        gov_core = builder.governors[i]

        gen_meta = builder.gen_metadata[i]
        exc_meta = builder.exc_metadata[i]
        gov_meta = builder.gov_metadata[i]

        gen_model = gen_core.model_name
        exc_model = exc_core.model_name
        gov_model = gov_core.model_name

        # Generator
        if gen_model in JIT_REGISTRY:
            reg = JIT_REGISTRY[gen_model]
            jit_data['gen_meta_arr'].append(reg['pack_meta'](gen_meta))
            jit_data['gen_dyn_jit'].append(reg['dynamics_jit'])
            jit_data['gen_ports_buf'].append(np.empty(reg['ports_size']))
        else:
            jit_data['gen_meta_arr'].append(None)
            jit_data['gen_dyn_jit'].append(None)
            jit_data['gen_ports_buf'].append(None)

        # Exciter
        if exc_model in JIT_REGISTRY:
            reg = JIT_REGISTRY[exc_model]
            jit_data['exc_meta_arr'].append(reg['pack_meta'](exc_meta))
            jit_data['exc_dyn_jit'].append(reg['dynamics_jit'])
            jit_data['exc_out_jit'].append(reg['output_jit'])
            jit_data['exc_ports_buf'].append(np.empty(reg['ports_size']))
        else:
            jit_data['exc_meta_arr'].append(None)
            jit_data['exc_dyn_jit'].append(None)
            jit_data['exc_out_jit'].append(None)
            jit_data['exc_ports_buf'].append(None)

        # Governor
        if gov_model in JIT_REGISTRY:
            reg = JIT_REGISTRY[gov_model]
            jit_data['gov_meta_arr'].append(reg['pack_meta'](gov_meta))
            jit_data['gov_dyn_jit'].append(reg['dynamics_jit'])
            jit_data['gov_out_jit'].append(reg['output_jit'])
            jit_data['gov_ports_buf'].append(np.empty(reg['ports_size']))
        else:
            jit_data['gov_meta_arr'].append(None)
            jit_data['gov_dyn_jit'].append(None)
            jit_data['gov_out_jit'].append(None)
            jit_data['gov_ports_buf'].append(None)

    return jit_data


def warmup_jit(jit_data, n_gen):
    """
    Trigger JIT compilation by calling each function once with dummy data.

    This avoids the first-call compilation penalty during the ODE solve.

    Args:
        jit_data: dict from prepare_jit_data()
        n_gen: number of generators
    """
    # Track which functions we've already warmed up (avoid double compilation)
    warmed = set()

    for i in range(n_gen):
        # Generator
        fn = jit_data['gen_dyn_jit'][i]
        if fn is not None and id(fn) not in warmed:
            meta = jit_data['gen_meta_arr'][i]
            x = np.zeros(7)
            p = np.zeros(len(jit_data['gen_ports_buf'][i]))
            fn(x, p, meta)
            warmed.add(id(fn))

        # Exciter dynamics + output
        fn = jit_data['exc_dyn_jit'][i]
        if fn is not None and id(fn) not in warmed:
            meta = jit_data['exc_meta_arr'][i]
            n_states = len(meta) - 5  # rough estimate, use actual
            # Determine state count from model
            exc_model = None
            for name, reg in JIT_REGISTRY.items():
                if reg['dynamics_jit'] is fn:
                    exc_model = name
                    break
            n_s = {'EXDC2': 4, 'EXST1': 4, 'ESST3A': 5, 'IEEEX1': 5}.get(exc_model, 4)
            x = np.zeros(n_s)
            p = np.zeros(len(jit_data['exc_ports_buf'][i]))
            fn(x, p, meta)
            warmed.add(id(fn))

        fn_out = jit_data['exc_out_jit'][i]
        if fn_out is not None and id(fn_out) not in warmed:
            meta = jit_data['exc_meta_arr'][i]
            exc_model = None
            for name, reg in JIT_REGISTRY.items():
                if reg.get('output_jit') is fn_out:
                    exc_model = name
                    break
            n_s = {'EXDC2': 4, 'EXST1': 4, 'ESST3A': 5, 'IEEEX1': 5}.get(exc_model, 4)
            x = np.zeros(n_s)
            p = np.zeros(len(jit_data['exc_ports_buf'][i]))
            fn_out(x, p, meta)
            warmed.add(id(fn_out))

        # Governor dynamics + output
        fn = jit_data['gov_dyn_jit'][i]
        if fn is not None and id(fn) not in warmed:
            meta = jit_data['gov_meta_arr'][i]
            gov_model = None
            for name, reg in JIT_REGISTRY.items():
                if reg['dynamics_jit'] is fn:
                    gov_model = name
                    break
            n_s = {'TGOV1': 2, 'IEEEG1': 6}.get(gov_model, 2)
            x = np.zeros(n_s)
            p = np.zeros(len(jit_data['gov_ports_buf'][i]))
            fn(x, p, meta)
            warmed.add(id(fn))

        fn_out = jit_data['gov_out_jit'][i]
        if fn_out is not None and id(fn_out) not in warmed:
            meta = jit_data['gov_meta_arr'][i]
            gov_model = None
            for name, reg in JIT_REGISTRY.items():
                if reg.get('output_jit') is fn_out:
                    gov_model = name
                    break
            n_s = {'TGOV1': 2, 'IEEEG1': 6}.get(gov_model, 2)
            x = np.zeros(n_s)
            p = np.zeros(len(jit_data['gov_ports_buf'][i]))
            fn_out(x, p, meta)
            warmed.add(id(fn_out))

    print(f"  JIT warmup complete: {len(warmed)} functions compiled")
