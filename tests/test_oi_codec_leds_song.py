from roomba_stack.l2_oi.oi_codec import encode_leds, encode_song, encode_play

def test_encode_leds_bits_only():
    # Debris on (bit0=1), others off; power color/intensity off
    assert encode_leds(True, False, False, False, 0, 0) == b"\x8b\x01\x00\x00"
    # Dock (bit2) and Check (bit3) on → bits 0b1100 = 12
    assert encode_leds(False, False, True, True, 0, 0) == b"\x8b\x0c\x00\x00"

def test_encode_leds_power_params():
    # Power LED green full: color=0, intensity=255
    assert encode_leds(False, False, False, False, 0, 255) == b"\x8b\x00\x00\xff"
    # Power LED red full: color=255, intensity=255
    assert encode_leds(False, False, False, False, 255, 255) == b"\x8b\x00\xff\xff"
    # Power LED orange half: color=128, intensity=128
    assert encode_leds(False, False, False, False, 128, 128) == b"\x8b\x00\x80\x80"

def test_encode_song_single_note():
    # song 0, 1 note: middle C (60) for 64/64 = 1.0s
    assert encode_song(0, [(60, 64)]) == b"\x8c\x00\x01\x3c\x40"

def test_encode_play():
    assert encode_play(0) == b"\x8d\x00"
