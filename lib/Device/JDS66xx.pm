package Device::JDS66xx;

use 5.032001;
use strict;
use warnings;

our $VERSION = '0.01';

use Carp;
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
  my ( $class, $df ) = @_;
  my $self = {};
  bless( $self , $class );
  $self->{devfile} = $df;
  $self->{n_try}   = 10;

  my $dobj = Device::SerialPort->new( $self->{devfile}, 0 )  || croak "Can't open $self->{devfile}: $!\n";

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
  my ($self, $cmd, $req ) = @_;
  my $cmdLen = length( $cmd );
  $req = $req || ':ok';
  my $reqLen = length( $req );

  my( $r_n, $rbuf );

  # carp( "sendCmd: \"$cmd\"" );

  for( my $i = 0; $i < $self->{n_try}; ++$i ) {
    my $wn = $self->{dev}->write( $cmd );
    if( $wn != $cmdLen ) {
      carp( "warning: bad send len: $wn != $cmdLen, cmd= $cmd\n" );
      next;
    }
    usleep( 10000 );

    ( $r_n, $rbuf ) = $self->{dev}->read( 255 );
    $rbuf =~ s/\r\n//x;

    if( length( $rbuf ) < $reqLen ) {
      next;
    }

    if( substr( $rbuf, 0, $reqLen ) eq $req ) {
      return $rbuf;
    }
  }

  carp( "warning: fail to send cmd= $cmd\n" );
  return;
}

sub setReg
{
  my ($self, $reg, $val ) = @_;

  # carp( "setReg $reg = $val" );
  my $s = sprintf( "\r\n:w%02d=%s.\r\n", $reg, $val );
  return $self->sendCmd( $s );
}

sub getReg
{
  my ($self, $reg) = @_;

  my $s = sprintf( "\r\n:r%02d=\r\n", $reg );
  my $rs = $self->sendCmd( $s, ':r' );

  # carp( "getReg: \"$rs\"\n" );

  if( $rs =~ m/^:r(\d\d)=([0-9,]+)\./x ) {
    my $reg_ret = 0 + $1;
    if( $reg != $reg_ret ) {
      return;
    }
    $rs = $2;
    return $rs;
  }
  return;
}

sub mkOnOffStr
{
  my ($self,$o12) = @_;
  my $o1  = ( $o12 & 1 ) ? '1' : '0'; #unpack
  my $o2  = ( $o12 & 2 ) ? '1' : '0';

  my $s   = $o1 . ',' . $o2;
  return $s;
}

sub OnOff
{
  my ($self, $o1, $o2, $checkNTry ) = @_;
  my $o12 = 0;
  if( $o1 ) { # pack
      $o12 |= 1;
  }
  if( $o2 ) {
      $o12 |= 2;
  }
  my $reg  = 20;
  my $fun = \&mkOnOffStr;
  return $self->setVal( $o12, $reg, $fun, $checkNTry );
}


sub mkWaveStr
{
  my ($self, $tp ) = @_;
  if( !defined( $tp ) ) {
    return;
  }
  $tp   = 0 + $tp;
  return '' . $tp;
}

sub setWave
{
  my ($self, $tp, $ch, $checkNTry ) = @_;
  my $reg  = $ch ? 22 : 21;
  my $fun = \&mkWaveStr;
  return $self->setVal( $tp, $reg, $fun, $checkNTry );
}

sub getWave
{
  my ($self, $ch ) = @_;
  my $reg  = $ch ? 22 : 21;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    return 0;
  }

  return 0 + $s;
}



sub mkFreqStr
{
  my ($self, $freq ) = @_;
  if( !defined( $freq ) ) {
    return;
  }
  $freq = 0.0 + $freq;
  my ($range, $fs );

  if( $freq < 80 ) {         # 80 Hz - in 10^{-2} * 10^{-6} Hz
    $range = '4';
    $fs = sprintf( "%d", int($freq * 1e8) );
  } elsif( $freq < 80000 ) { # 80 kHz - in 10^{-2} * 10^{-3} Hz
    $range = '3';
    $fs = sprintf( "%d", int($freq * 1e5) );
  } else {                   # > 80kHz ; in 10^{-2} Hz
    $range = '0';
    $fs = sprintf( "%d", int($freq * 100) );
  }

  return $fs . ',' . $range;
}

sub setVal
{
  my ($self, $val, $reg, $mkStrFun, $checkNTry ) = @_;
  my $vs = $self->$mkStrFun( $val );
  if( ! $vs ) {
    return;
  }

  if( ! $checkNTry ) {
    $checkNTry = 10;
  }

  TRY:
  for my $i (1..$checkNTry) {
    my $rs =  $self->setReg( $reg, $vs );
    if( ! $rs ) {
      usleep( $self->{wait_after_bad_set} );
      next TRY;
      # debug?
    }
    my $gs = $self->getReg( $reg );
    if( defined($gs) && ( $gs eq $vs ) ) {
      return $vs;
    } else {
      usleep( $self->{wait_after_bad_set} );
      carp( "setVal: get ($gs) != set ($vs) i= $i" );
    }
  }

  return;
}


sub setFreq
{
  my ($self, $freq, $ch, $checkNTry ) = @_;
  my $reg  = $ch ? 24 : 23;
  my $fun = \&mkFreqStr;
  return $self->setVal( $freq, $reg, $fun, $checkNTry );
}

sub getFreq
{
  my ($self, $ch ) = @_;
  my $reg  = $ch ? 24 : 23;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    carp( "Fail to get reg $reg" );
    return 0.0;
  }

  my $fs   = 0;
  my $r    = 0;

  if( $s =~ /^(\d+),(\d)/x  ) {
    $fs   = 0 + $1;
    $r    = 0 + $2;
  } else {
    carp( "Bad freq responce string: \"$s\"" );
    return 0.0;
  }

  my $freq = 0.0;

  my %ranges = (
      0 => 1.0e-2,
      1 => 1.0e+1,
      2 => 1.0e+4,
      3 => 1.0e-5,
      4 => 1.0e-8
  );

  if( defined( $ranges{$r} ) ) {
     $freq = $fs * $ranges{$r};
  }

  return $freq;
}

sub setFreqCheck # TODO: remove, replace with setFreq with checkNTry
{
  my ($self, $f, $ch ) = @_;

  for( my $i=0; $i<$self->{n_try}; ++$i ) {
    my $rc = $self->setFreq( $f, $ch );
    if( ! $rc ) {
      carp( "?1 \n" ); # TODO: hide w/o debug?
      usleep( $self->{wait_after_bad_set} );
      next;
    }
    my $f_in = $self->getFreq( $ch );
    if( abs( $f_in - $f ) < 10 ) {
      return 1;
    }
  carp( "?2 f= $f f_in= $f_in \n" ); #  TODO: hide w/o debug?
  usleep( $self->{wait_after_bad_set} );
  }

  return 0;
}

sub mkVppStr
{
  my ($self, $v ) = @_;
  if( !defined( $v ) ) {
    return;
  }
  my $vi    = 1000.0 * (0.0 + $v);

  my $s = sprintf( "%d", int($vi) );

  return $s;
}

sub setVpp
{
  my ($self, $v, $ch, $checkNTry ) = @_;
  my $reg  = $ch ? 26 : 25;
  my $fun = \&mkVppStr;
  return $self->setVal( $v, $reg, $fun, $checkNTry );
}

sub getVpp
{
  my ($self, $ch ) = @_;
  my $reg  = $ch ? 26 : 25;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    return 0;
  }

  return (0.0+$s) / 1000.0;
}


sub mkBiasStr
{
  my ($self, $bias ) = @_;
  if( !defined( $bias ) ) {
    return;
  }
  my $bi   = 1000 + 100.0 * (0.0 + $bias);
  my $s = sprintf( "%d", int($bi) );
  return $s;
}


sub setBias
{
  my ($self, $bias, $ch, $checkNTry ) = @_;
  my $reg  = $ch ? 28 : 27;
  my $fun = \&mkBiasStr;
  return $self->setVal( $bias, $reg, $fun, $checkNTry );
}

sub getBias
{
  my ($self, $ch ) = @_;
  my $reg  = $ch ? 28 : 27;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    return 0.0;
  }

  if( $s =~ /^(\d+)/x ) {
    return ( -10.0 + 0.01 * $1 );
  }

  return 0.0;
}

sub mkDutyStr
{
  my ($self, $du) = @_;
  if( !defined( $du ) ) {
    return;
  }
  my $dui   = 1000.0 * (0.0 + $du);
  my $s = sprintf( "%d", int($dui) );
  return $s;
}



sub setDuty
{
  my ($self, $du, $ch, $checkNTry ) = @_;
  my $reg  = $ch ? 30 : 29;
  my $fun = \&mkDutyStr;
  return $self->setVal( $du, $reg, $fun, $checkNTry );
}

sub getDuty
{
  my ($self, $ch ) = @_;
  my $reg  = $ch ? 30 : 29;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    return 0.0;
  }

  if( $s =~ /^(\d+)/x ) {
    return ( 0.001 * $1 );
  }

  return 0.0;
}

sub setDutyCheck # TODO: remove
{
  my ($self, $du, $ch ) = @_;

  for( my $i=0; $i<$self->{n_try}; ++$i ) {
    my $rc = $self->setDuty( $du, $ch );
    if( ! $rc ) {
      carp( "?1 \n" );
      usleep( $self->{wait_after_bad_set} );
      next;
    }
    my $du_in = $self->getDuty( $ch );
    if( abs( $du - $du_in ) < 0.01 ) {
      return 1;
    }
  carp( "?2 du= $du du_in = $du_in\n" );
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
it under the  terms of GPLV3.


=cut
