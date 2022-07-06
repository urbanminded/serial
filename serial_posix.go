//go:build !windows && !linux && cgo
// +build !windows,!linux,cgo

package serial

// #include <termios.h>
// #include <unistd.h>
import "C"

// TODO: Maybe change to using syscall package + ioctl instead of cgo

import (
	"errors"
	"fmt"
	"os"
	"syscall"
	"time"
	//"unsafe"
)

var bauds = map[int]C.speed_t{
	50:     C.B50,
	75:     C.B75,
	110:    C.B110,
	134:    C.B134,
	150:    C.B150,
	200:    C.B200,
	300:    C.B300,
	600:    C.B600,
	1200:   C.B1200,
	2400:   C.B2400,
	4800:   C.B4800,
	9600:   C.B9600,
	19200:  C.B19200,
	38400:  C.B38400,
	57600:  C.B57600,
	115200: C.B115200,
}

func isStandardBaudRate(baud int) bool {
	_, ok := bauds[baud]
	return ok
}

func getStandardBaudRate(baud int) C.speed_t {
	if rate, ok := bauds[baud]; ok {
		return rate
	}
	return 0
}

func openPort(name string, baud int, databits byte, parity Parity, stopbits StopBits, readTimeout time.Duration) (p *Port, err error) {
	f, err := os.OpenFile(name, syscall.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0666)
	if err != nil {
		return
	}

	fd := C.int(f.Fd())
	if C.isatty(fd) != 1 {
		f.Close()
		return nil, errors.New("File is not a tty")
	}

	var st C.struct_termios
	_, err = C.tcgetattr(fd, &st)
	if err != nil {
		f.Close()
		return nil, err
	}

	// This may seem a bit strange but the POSIX API doesn't explicitly state that
	// baud rate constants are literal baud rate values.
	speed := getStandardBaudRate(baud)
	if speed == 0 {
		speed = C.speed_t(baud)
	}

	_, err = C.cfsetispeed(&st, speed)
	if err != nil {
		f.Close()
		return nil, err
	}
	_, err = C.cfsetospeed(&st, speed)
	if err != nil {
		f.Close()
		return nil, err
	}

	// Turn off break interrupts, CR->NL, Parity checks, strip, and IXON
	st.c_iflag &= ^C.tcflag_t(C.BRKINT | C.ICRNL | C.INPCK | C.ISTRIP | C.IXOFF | C.IXON | C.PARMRK)

	// Select local mode, turn off parity, set to 8 bits
	st.c_cflag &= ^C.tcflag_t(C.CSIZE | C.PARENB)
	st.c_cflag |= (C.CLOCAL | C.CREAD)
	// databits
	switch databits {
	case 5:
		st.c_cflag |= C.CS5
	case 6:
		st.c_cflag |= C.CS6
	case 7:
		st.c_cflag |= C.CS7
	case 8:
		st.c_cflag |= C.CS8
	default:
		return nil, ErrBadSize
	}
	// Parity settings
	switch parity {
	case ParityNone:
		// default is no parity
	case ParityOdd:
		st.c_cflag |= C.PARENB
		st.c_cflag |= C.PARODD
	case ParityEven:
		st.c_cflag |= C.PARENB
		st.c_cflag &= ^C.tcflag_t(C.PARODD)
	default:
		return nil, ErrBadParity
	}
	// Stop bits settings
	switch stopbits {
	case Stop1:
		// as is, default is 1 bit
	case Stop2:
		st.c_cflag |= C.CSTOPB
	default:
		return nil, ErrBadStopBits
	}
	// Select raw mode
	st.c_lflag &= ^C.tcflag_t(C.ICANON | C.ECHO | C.ECHOE | C.ISIG)
	st.c_oflag &= ^C.tcflag_t(C.OPOST)

	// set blocking / non-blocking read
	/*
	*	http://man7.org/linux/man-pages/man3/termios.3.html
	* - Supports blocking read and read with timeout operations
	 */
	vmin, vtime := posixTimeoutValues(readTimeout)
	st.c_cc[C.VMIN] = C.cc_t(vmin)
	st.c_cc[C.VTIME] = C.cc_t(vtime)

	_, err = C.tcsetattr(fd, C.TCSANOW, &st)
	if err != nil {
		f.Close()
		return nil, err
	}

	//fmt.Println("Tweaking", name)
	r1, _, e := syscall.Syscall(syscall.SYS_FCNTL,
		uintptr(f.Fd()),
		uintptr(syscall.F_SETFL),
		uintptr(0))
	if e != 0 || r1 != 0 {
		s := fmt.Sprint("Clearing NONBLOCK syscall error:", e, r1)
		f.Close()
		return nil, errors.New(s)
	}

	/*
				r1, _, e = syscall.Syscall(syscall.SYS_IOCTL,
			                uintptr(f.Fd()),
			                uintptr(0x80045402), // IOSSIOSPEED
			                uintptr(unsafe.Pointer(&baud)));
			        if e != 0 || r1 != 0 {
			                s := fmt.Sprint("Baudrate syscall error:", e, r1)
					f.Close()
		                        return nil, os.NewError(s)
				}
	*/

	return &Port{f: f}, nil
}

type Port struct {
	// We intentionly do not use an "embedded" struct so that we
	// don't export File
	f *os.File
}

func (p *Port) Read(b []byte) (n int, err error) {
	return p.f.Read(b)
}

func (p *Port) Write(b []byte) (n int, err error) {
	return p.f.Write(b)
}

// Discards data written to the port but not transmitted,
// or data received but not read
func (p *Port) Flush() error {
	_, err := C.tcflush(C.int(p.f.Fd()), C.TCIOFLUSH)
	return err
}

func (p *Port) Close() (err error) {
	return p.f.Close()
}
