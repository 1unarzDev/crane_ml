"""Which GPU a Windows player renders on.

A Windows host with an integrated and a discrete GPU (simwin: Intel UHD 770 and an RTX 3060) lists the
adapter driving the desktop first, and a Unity player takes DXGI adapter 0 unless told otherwise; in the
background (session 0) that put the players on the Intel GPU, which crashed in its driver. Neither the
per-app preference (``HKCU\\Software\\Microsoft\\DirectX\\UserGpuPreferences``, which install.ps1 still sets)
nor anything else moves a Unity 6 player off adapter 0, so the backends pass ``-force-device-index`` with
the DXGI index of the adapter with the most dedicated video memory. ``player_gpu_args`` returns those
arguments (none on other platforms, with one adapter, or when the job's arguments already pick a device).
"""

import ctypes
import functools
import sys

DXGI_ERROR_NOT_FOUND = 0x887A0002
SOFTWARE = 2   # DXGI_ADAPTER_FLAG_SOFTWARE (the Basic Render Driver)


@functools.lru_cache(maxsize=1)
def dxgi_adapters():
    """[(index, description, dedicated video memory in bytes, software)] in DXGI order; [] when unavailable."""
    if sys.platform != 'win32':
        return ()
    from ctypes import wintypes

    class GUID(ctypes.Structure):
        _fields_ = [('d1', ctypes.c_ulong), ('d2', ctypes.c_ushort), ('d3', ctypes.c_ushort), ('d4', ctypes.c_ubyte * 8)]

    class DESC1(ctypes.Structure):
        _fields_ = [('Description', wintypes.WCHAR * 128), ('VendorId', wintypes.UINT), ('DeviceId', wintypes.UINT),
                    ('SubSysId', wintypes.UINT), ('Revision', wintypes.UINT), ('DedicatedVideoMemory', ctypes.c_size_t),
                    ('DedicatedSystemMemory', ctypes.c_size_t), ('SharedSystemMemory', ctypes.c_size_t),
                    ('LuidLow', wintypes.DWORD), ('LuidHigh', wintypes.LONG), ('Flags', wintypes.UINT)]

    def method(obj, index, *argtypes):   # a COM method from the object's vtable
        vtable = ctypes.cast(obj, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p)))[0]
        return ctypes.WINFUNCTYPE(ctypes.c_long, ctypes.c_void_p, *argtypes)(vtable[index])

    iid = GUID(0x770aae78, 0xf26f, 0x4dba, (ctypes.c_ubyte * 8)(0xa8, 0x29, 0x25, 0x3c, 0x83, 0xd1, 0xb3, 0x87))  # IDXGIFactory1
    factory = ctypes.c_void_p()
    try:
        if ctypes.windll.dxgi.CreateDXGIFactory1(ctypes.byref(iid), ctypes.byref(factory)) != 0:
            return ()
    except OSError:
        return ()
    out = []
    try:
        for i in range(16):
            adapter = ctypes.c_void_p()
            hr = method(factory, 12, wintypes.UINT, ctypes.POINTER(ctypes.c_void_p))(factory, i, ctypes.byref(adapter))  # EnumAdapters1
            if hr & 0xFFFFFFFF == DXGI_ERROR_NOT_FOUND or hr != 0:
                break
            d = DESC1()
            try:
                if method(adapter, 10, ctypes.POINTER(DESC1))(adapter, ctypes.byref(d)) == 0:   # GetDesc1
                    out.append((i, d.Description, int(d.DedicatedVideoMemory), bool(d.Flags & SOFTWARE)))
            finally:
                method(adapter, 2)(adapter)   # Release
    finally:
        method(factory, 2)(factory)
    return tuple(out)


def discrete_index(adapters):
    """The index of the hardware adapter with the most dedicated memory when there are several, else None."""
    hw = [a for a in adapters if not a[3]]
    if len(hw) < 2:
        return None
    return max(hw, key=lambda a: (a[2], -a[0]))[0]


def player_gpu_args(args, adapters=None):
    """``-force-device-index N`` for the discrete GPU of a Windows host with several, unless ``args`` pick one."""
    if any(str(a) in ('-force-device-index', '-gpu') for a in args):
        return []
    idx = discrete_index(dxgi_adapters() if adapters is None else adapters)
    return [] if idx is None else ['-force-device-index', str(idx)]


def hidden_desktop(env):
    """The hidden desktop the gpu lane starts players on (``CRANE_DESKTOP``), or None."""
    return env.get('CRANE_DESKTOP') if sys.platform == 'win32' else None


def start_player(argv, cwd, env):
    """Starts a player at below-normal priority with no console window, detached from the worker's signals.
    In the gpu lane (Windows, the user's desktop session) it starts on the lane's hidden desktop
    (crane_cluster/hiddendesk.py), so nothing it opens can be seen or take focus."""
    import subprocess
    if sys.platform != 'win32':
        return subprocess.Popen(argv, cwd=cwd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                start_new_session=True)
    flags = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW | subprocess.BELOW_NORMAL_PRIORITY_CLASS
    desk = hidden_desktop(env)
    if desk:
        from crane_cluster import hiddendesk
        return hiddendesk.popen(argv, desk, cwd=cwd, env=env, creationflags=flags)
    si = subprocess.STARTUPINFO()
    si.dwFlags |= subprocess.STARTF_USESHOWWINDOW  # SW_HIDE for any window the player makes
    return subprocess.Popen(argv, cwd=cwd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            startupinfo=si, creationflags=flags)
