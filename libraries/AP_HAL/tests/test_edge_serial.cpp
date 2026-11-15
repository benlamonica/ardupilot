#include <AP_gtest.h>
#include <AP_HAL/utility/EdgeSerial.h>

#include <vector>

using namespace AP_HAL::EdgeSerial;

namespace {

const uint32_t BAUD = 19200;

// a queue of edges in the shape decode_frame() reads
class EdgeQueue {
public:
    std::vector<Edge> edges;
    size_t head;

    EdgeQueue() : head(0) {}
    bool available() const { return head < edges.size(); }
    const Edge &peek() const { return edges[head]; }
    void pop() { head++; }

    // the next falling edge, as the receiver looks for a start bit
    bool next_start(uint32_t &t0)
    {
        while (available()) {
            const Edge e = peek();
            pop();
            if (e.level == 0) {
                t0 = e.t_us;
                return true;
            }
        }
        return false;
    }
};

/*
  Simple deterministic pseudo random numbers, so a failure can be repeated.
 */
class Rand {
public:
    explicit Rand(uint32_t seed) : state(seed) {}
    // uniform in [-1, 1]
    double next()
    {
        state = state * 1664525U + 1013904223U;
        return (double)(state >> 8) / (double)(1U << 23) - 1.0;
    }
private:
    uint32_t state;
};

/*
  Line edges for bytes sent back to back by a sender whose bit time is
  bit_us, starting at t_start. Each edge is moved by up to jitter_us, as
  interrupt latency would. A line that ends a frame low (a missing stop bit)
  is given by stop_level 0.
 */
std::vector<Edge> make_edges(const std::vector<uint8_t> &bytes, double bit_us, double t_start,
                             double jitter_us = 0, uint32_t seed = 1, uint8_t stop_level = 1)
{
    std::vector<Edge> edges;
    Rand rnd(seed);
    uint8_t level = 1;
    double t = t_start;
    for (uint8_t b : bytes) {
        uint16_t bits = frame_bits(b);
        if (!stop_level) {
            bits &= ~(1U << 9);
        }
        for (uint8_t i = 0; i < 10; i++) {
            const uint8_t bit = (bits >> i) & 1U;
            if (bit != level) {
                const double tj = t + jitter_us * rnd.next();
                edges.push_back({(uint32_t)(tj + 0.5), bit});
                level = bit;
            }
            t += bit_us;
        }
    }
    if (level == 0) {
        edges.push_back({(uint32_t)(t + 0.5), 1});
    }
    return edges;
}

// decode everything in the queue the way RCOutput_pico_serial reads a reply
std::vector<uint8_t> decode_all(EdgeQueue &q, uint32_t &framing_errors)
{
    const uint32_t bt = bit_time_x16(BAUD);
    std::vector<uint8_t> out;
    framing_errors = 0;
    uint32_t t0;
    while (q.next_start(t0)) {
        uint8_t byte;
        if (!decode_frame(t0, q, bt, byte)) {
            framing_errors++;
            break;
        }
        out.push_back(byte);
    }
    return out;
}

std::vector<uint8_t> all_bytes()
{
    std::vector<uint8_t> v;
    for (uint16_t i = 0; i < 256; i++) {
        v.push_back((uint8_t)i);
    }
    return v;
}

const double BIT_US = 1e6 / BAUD;

}  // namespace

TEST(EdgeSerial, BitTime)
{
    // 52.083 us at 19200 baud, kept to 1/16 us
    EXPECT_EQ(bit_time_x16(19200), 833U);
    EXPECT_EQ(half_bits_us(2, bit_time_x16(19200)), 52U);
    EXPECT_EQ(half_bits_us(20, bit_time_x16(19200)), 521U);
    EXPECT_EQ(rx_frame_us(bit_time_x16(19200)), 521U);
}

TEST(EdgeSerial, FrameBits)
{
    // start bit low, data LSB first, stop bit high
    EXPECT_EQ(frame_bits(0x00), 0x200U);
    EXPECT_EQ(frame_bits(0xFF), 0x3FEU);
    EXPECT_EQ(frame_bits(0x30), 0x260U);
}

TEST(EdgeSerial, TransmitSchedule)
{
    const uint32_t bt = bit_time_x16(BAUD);
    // each bit ends one bit time after the previous one
    for (uint8_t bit = 0; bit < 10; bit++) {
        EXPECT_EQ(tx_bit_end_us(bit, false, bt), half_bits_us(2U * (bit + 1U), bt));
    }
    // all ten bits take 10 bit times, except that the final stop bit of a
    // write is held only to its middle, 9.5 bit times
    EXPECT_EQ(tx_bit_end_us(9, false, bt), 521U);
    EXPECT_EQ(tx_bit_end_us(9, true, bt), 495U);
    // only the stop bit is shortened
    EXPECT_EQ(tx_bit_end_us(8, true, bt), tx_bit_end_us(8, false, bt));
}

TEST(EdgeSerial, CleanBytes)
{
    EdgeQueue q;
    const std::vector<uint8_t> bytes = all_bytes();
    q.edges = make_edges(bytes, BIT_US, 1000);
    uint32_t fe;
    EXPECT_EQ(decode_all(q, fe), bytes);
    EXPECT_EQ(fe, 0U);
}

TEST(EdgeSerial, BackToBackWithGaps)
{
    // a reply in bursts, with idle time between some bytes
    const std::vector<uint8_t> bytes = {0x34, 0x37, 0x31, 0xE8, 0xB2, 0x64, 0x01, 0x30};
    EdgeQueue q;
    double t = 5000;
    for (size_t i = 0; i < bytes.size(); i++) {
        const std::vector<Edge> e = make_edges({bytes[i]}, BIT_US, t);
        q.edges.insert(q.edges.end(), e.begin(), e.end());
        t += 10 * BIT_US + (i % 3) * 37.0;
    }
    uint32_t fe;
    EXPECT_EQ(decode_all(q, fe), bytes);
    EXPECT_EQ(fe, 0U);
}

TEST(EdgeSerial, SenderOffBaud)
{
    // ESC bootloaders run from an internal oscillator: allow +-5%
    const std::vector<uint8_t> bytes = all_bytes();
    for (double err : {-0.05, -0.03, 0.03, 0.05}) {
        EdgeQueue q;
        q.edges = make_edges(bytes, BIT_US * (1.0 + err), 1000);
        uint32_t fe;
        EXPECT_EQ(decode_all(q, fe), bytes) << "baud error " << err;
        EXPECT_EQ(fe, 0U) << "baud error " << err;
    }
}

TEST(EdgeSerial, EdgeJitter)
{
    // edges timestamped late or early by interrupt latency, up to 15% of a bit
    const std::vector<uint8_t> bytes = all_bytes();
    for (uint32_t seed = 1; seed <= 20; seed++) {
        EdgeQueue q;
        q.edges = make_edges(bytes, BIT_US, 1000, 0.15 * BIT_US, seed);
        uint32_t fe;
        EXPECT_EQ(decode_all(q, fe), bytes) << "seed " << seed;
        EXPECT_EQ(fe, 0U) << "seed " << seed;
    }
}

TEST(EdgeSerial, JitterAndOffBaud)
{
    const std::vector<uint8_t> bytes = all_bytes();
    for (double err : {-0.03, 0.03}) {
        for (uint32_t seed = 1; seed <= 10; seed++) {
            EdgeQueue q;
            q.edges = make_edges(bytes, BIT_US * (1.0 + err), 1000, 0.10 * BIT_US, seed);
            uint32_t fe;
            EXPECT_EQ(decode_all(q, fe), bytes) << "baud error " << err << " seed " << seed;
        }
    }
}

TEST(EdgeSerial, EarlyStartAtRelease)
{
    /*
      The ESC began its start bit while we still drove the line, so its start
      edge is never seen: the receiver finds the line already low at release
      and takes early_start_us() as the start. The ESC may begin from the
      middle of our stop bit, which is where we release, and an ESC clock 5%
      fast moves that up to half a bit (25 us at 19200 baud) earlier. Every
      byte, the bootloader ack (0x30) included, must decode for any start up
      to 0.7 bit (36 us) before release, which leaves a margin.
     */
    const uint32_t bt = bit_time_x16(BAUD);
    const std::vector<uint8_t> bytes = all_bytes();
    for (uint32_t before_release_us = 0; before_release_us <= 36; before_release_us++) {
        for (uint8_t b : bytes) {
            const double start = 1000;
            const uint32_t release = (uint32_t)start + before_release_us;
            EdgeQueue q;
            q.edges = make_edges({b}, BIT_US, start);
            ASSERT_EQ(q.edges[0].level, 0);
            // the real start edge is lost, and a stand-in is put at release
            q.edges[0].t_us = early_start_us(release, bt);
            // drop any edge that came before release: the line was driven then
            while (q.edges.size() > 1 && q.edges[1].t_us <= release) {
                q.edges.erase(q.edges.begin() + 1);
            }
            uint32_t fe;
            const std::vector<uint8_t> out = decode_all(q, fe);
            ASSERT_EQ(out.size(), 1U) << "byte " << unsigned(b) << " started " << before_release_us << " us before release";
            EXPECT_EQ(out[0], b) << "started " << before_release_us << " us before release";
            EXPECT_EQ(fe, 0U);
        }
    }
}

TEST(EdgeSerial, EarlyStartTime)
{
    // a quarter bit, 13 us at 19200 baud, before release
    EXPECT_EQ(early_start_us(1000, bit_time_x16(19200)), 987U);
}

TEST(EdgeSerial, MissingStopBit)
{
    // a frame whose stop bit is low is a framing error, and the read stops
    EdgeQueue q;
    q.edges = make_edges({0x55}, BIT_US, 1000, 0, 1, 0);
    uint32_t fe;
    EXPECT_TRUE(decode_all(q, fe).empty());
    EXPECT_EQ(fe, 1U);
}

TEST(EdgeSerial, BreakIsFramingError)
{
    // the line held low for longer than a frame
    EdgeQueue q;
    q.edges = {{1000, 0}, {1000 + 30 * 52, 1}};
    uint32_t fe;
    EXPECT_TRUE(decode_all(q, fe).empty());
    EXPECT_EQ(fe, 1U);
}

TEST(EdgeSerial, StopsAtFramingErrorKeepsEarlierBytes)
{
    // good bytes before a bad one are kept
    EdgeQueue q;
    q.edges = make_edges({0x2E, 0x37}, BIT_US, 1000);
    const std::vector<Edge> bad = make_edges({0x99}, BIT_US, 1000 + 25 * BIT_US, 0, 1, 0);
    q.edges.insert(q.edges.end(), bad.begin(), bad.end());
    uint32_t fe;
    const std::vector<uint8_t> expect = {0x2E, 0x37};
    EXPECT_EQ(decode_all(q, fe), expect);
    EXPECT_EQ(fe, 1U);
}

AP_GTEST_MAIN()
