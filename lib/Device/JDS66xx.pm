package Device::JDS66xx;

use 5.032001;
use strict;
use warnings;

our $VERSION = '0.01';

use Device::SerialPort;
use Time::HiRes qw( usleep );


use constant {
    SIN         =>  0,
    SINE        =>  0,
    SQUARE      =>  1,
    PULSE       =>  2,
    TRIANGLE    =>  3,
    PARTIALSINE =>  4,
    CMOS        =>  5,
    DC          =>  6,
    HALF_WAVE   =>  7,
    FULL_WAVE   =>  8,
    POS_LADDER  =>  9,
    NEG_LADDER  => 10,
    NOISE       => 11,
    EXP_RIZE    => 12,
    EXP_DECAY   => 13,
    MULTI_TONE  => 14,
    SINC        => 15,
    LORENZ      => 16
};

sub new
{
  my $class = shift;
  my $self = {};
  bless( $self , $class );
  $self->{devfile} = $_[0];
  $self->{n_try}   = 10;

  my $dobj = new Device::SerialPort( $self->{devfile}, 0 )  || die "Can't open $self->{devfile}: $!\n";

  $dobj->{"_DEBUG"} = 1;
  $dobj->devicetype( 'none' );
  $dobj->datatype( 'raw' );
  $dobj->error_msg( 'ON' );
  $dobj->baudrate( 115200 );
  $dobj->parity( 'none' );
  $dobj->databits( 8 );
  $dobj->stopbits( 1 );
  $dobj->handshake( 'none' );
  $dobj->alias( 'JDS' );
  $self->{dev} = $dobj;
  $self->{wait_after_bad_set} = 100000;
  return $self;
}

sub getDev
{
  my $self = shift;
  return $self->{dev};
}

sub sendCmd
{
  my $self = shift;
  my $cmd = $_[0];
  my $cmdLen = length( $cmd );
  my $req = $_[1] || ':ok';
  my $reqLen = length( $req );

  my( $r_n, $rbuf );

  for( my $i = 0; $i < $self->{n_try}; ++$i ) {
    my $wn = $self->{dev}->write( $cmd );
    if( $wn != $cmdLen ) {
      print( STDERR "warning: bad send len: $wn != $cmdLen, cmd= $cmd\n" );
      next;
    }
    usleep( 10000 );

    ( $r_n, $rbuf ) = $self->{dev}->read( 255 );
    $rbuf =~ s/\r\n//;

    if( length( $rbuf ) < $reqLen ) {
      next;
    }

    if( substr( $rbuf, 0, $reqLen ) eq $req ) {
      return $rbuf;
    }
  }

  print( STDERR "warning: fail to send cmd= $cmd\n" );
  return '';
}

sub setReg
{
  my $self = shift;
  my $reg  = $_[0];
  my $val  = $_[1];

  my $s = sprintf( "\r\n:w%02d=%s.\r\n", $reg, $val );
  return $self->sendCmd( $s );
}

sub getReg
{
  my $self = shift;
  my $reg  = $_[0];

  my $s = sprintf( "\r\n:r%02d=\r\n", $reg );
  my $rs = $self->sendCmd( $s, ':r' );
  $rs =~ s/^:r\d\d=//;
  return $rs;
}

sub OnOff
{
  my $self = shift;
  my $o1  = $_[0] ? '1' : '0';
  my $o2  = $_[1] ? '1' : '0';
  my $s   = $o1 . ',' . $o2;

  return $self->setReg( 20, $s );
}

sub setWave
{
  my $self = shift;
  if( !defined( $_[0] ) ) {
    return '';
  }
  my $tp   = 0 + $_[0];
  my $reg  = $_[1] ? 22 : 21;

  return $self->setReg( $reg, '' . $tp );
}


sub setFreq
{
  my $self = shift;
  if( !defined( $_[0] ) ) {
    return '';
  }
  my $freq = 0.0 + $_[0];
  my $reg  = $_[1] ? 24 : 23;
  my $range = '0';
  my $fs = sprintf( "%d", int($freq * 100) );

  if( $freq < 80 ) {
    $range = '4';
    $fs = sprintf( "%d", int($freq * 1e8) );
  } elsif( $freq < 80000 ) {
    $range = '3';
    $fs = sprintf( "%d", int($freq * 1e5) );
  }

  return $self->setReg( $reg, $fs . ',' . $range );
}

sub getFreq
{
  my $self = shift;
  my $reg  = $_[0] ? 24 : 23;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    return 0.0;
  }
  if( ! ( $s =~ /^(\d+),(\d)/ ) ) {
    return 0.0;
  }

  my $fs   = 0 + $1;
  my $r    = 0 + $2;
  my $freq = 0.0;

  if( $r == 0 ) {
      $freq = 1.0e-2 * $fs;
  } elsif ( $r == 1  ) {
      $freq = 1.0e+1  * $fs;
  } elsif ( $r == 2  ) {
      $freq = 1.0e+4  * $fs;
  } elsif ( $r == 3  ) {
      $freq = 1.0e-5 * $fs;
  } elsif ( $r == 4  ) {
      $freq = 1.0e-8 * $fs;
  }

  return $freq;
}

sub setFreqCheck
{
  my $self = shift;
  my $f  = $_[0];
  my $ch = $_[1];
  for( my $i=0; $i<$self->{n_try}; ++$i ) {
    my $rc = $self->setFreq( $f, $ch );
    if( ! $rc ) {
      print( STDERR "?1 \n" );
      usleep( $self->{wait_after_bad_set} );
      next;
    }
    my $f_in = $self->getFreq( $ch );
    if( abs( $f_in - $f ) < 10 ) {
      return 1;
    }
  print( STDERR "?2 \n" );
  usleep( $self->{wait_after_bad_set} );
  }
  return 0;
}

sub setVpp
{
  my $self = shift;
  if( !defined( $_[0] ) ) {
    return '';
  }
  my $v    = 1000.0 * (0.0 + $_[0]);
  my $reg  = $_[1] ? 26 : 25;

  my $s = sprintf( "%d", int($v) );

  return $self->setReg( $reg, $s );
}


sub setBias
{
  my $self = shift;
  if( !defined( $_[0] ) ) {
    return '';
  }
  my $v    = 1000 + 100.0 * (0.0 + $_[0]);
  my $reg  = $_[1] ? 28 : 27;

  my $s = sprintf( "%d", int($v) );

  return $self->setReg( $reg, $s );
}


sub setDuty
{
  my $self = shift;
  if( !defined( $_[0] ) ) {
    return '';
  }
  my $v    = 1000.0 * (0.0 + $_[0]);
  my $reg  = $_[1] ? 30 : 29;

  my $s = sprintf( "%d", int($v) );

  return $self->setReg( $reg, $s );
}

sub getDuty
{
  my $self = shift;
  my $reg  = $_[0] ? 30 : 29;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    return 0.0;
  }
  if( ! ( $s =~ /^(\d+)\./ ) ) {
    return 0.0;
  }

  my $du   = 0.001 * $1;

  return $du;
}

sub setDutyCheck
{
  my $self = shift;
  my $du = $_[0];
  my $ch = $_[1];
  for( my $i=0; $i<$self->{n_try}; ++$i ) {
    my $rc = $self->setDuty( $du, $ch );
    if( ! $rc ) {
      print( STDERR "?1 \n" );
      usleep( $self->{wait_after_bad_set} );
      next;
    }
    my $du_in = $self->getDuty( $ch );
    if( abs( $du - $du_in ) < 0.01 ) {
      return 1;
    }
  print( STDERR "?2 \n" );
  usleep( $self->{wait_after_bad_set} );
  }
  return 0;
}




# Preloaded methods go here.

1;

__END__

=head1 NAME

Device::JDS66xx - Perl module to control signal generator like JDS6600

=head1 SYNOPSIS

  use Device::JDS66xx;
  my $jds = new Device::JDS66xx( '/dev/ttyUSB0' );
  $jds->OnOff( 0, 0 );
  $jds->OnOff( 1, 1 );
  $jds->setFreq( 12345678.91, 0 );
  $jds->setFreq( 1234.56789,  1 );
  printf( "Freq: %f %f\n", $jds->getFreq(0), $jds->getFreq(1) );
  $jds->setVpp( 5.1234,  0 );
  $jds->setVpp( 1.23456, 1 );

  $jds->setBias( +1.23456, 0 );
  $jds->setBias( -1.23456, 1 );

  $jds->setDuty( 0.1, 0 );
  $jds->setDuty( 0.9, 1 );

  # manual
  my $rs = $jds->getReg( 0 );
  print( "rs=\"$rs\"\n" );
  $rs = $jds->setReg( 23, "1234567891,0" );
  $rs = $jds->sendCmd( "\r\n:w23=1234567891,0.\r\n" );


=head1 DESCRIPTION

This module allows us to control JDX6600 alike signal generator via serial port.
In creation state - so no documentation.


=head2 EXPORT

None by default.



=head1 SEE ALSO

Mention other useful documentation

=head1 AUTHOR

Anton Guda, E<lt>atu@nmetau.edu.uaE<gt>

=head1 COPYRIGHT AND LICENSE

Copyright (C) 2021 by Anton Guda

This library is free software; you can redistribute it and/or modify
it under the same terms as Perl itself, either Perl version 5.32.1 or,
at your option, any later version of Perl 5 you may have available.


=cut
