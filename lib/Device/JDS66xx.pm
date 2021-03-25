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

  for( my $i = 0; $i < $self->{n_try}; ++$i ) {
    my $wn = $self->{dev}->write( $cmd );
    if( $wn != $cmdLen ) {
      print( STDERR "warning: bad send len: $wn != $cmdLen, cmd= $cmd\n" );
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

  print( STDERR "warning: fail to send cmd= $cmd\n" );
  return '';
}

sub setReg
{
  my ($self, $reg, $val ) = @_;

  my $s = sprintf( "\r\n:w%02d=%s.\r\n", $reg, $val );
  return $self->sendCmd( $s );
}

sub getReg
{
  my ($self, $reg) = @_;

  my $s = sprintf( "\r\n:r%02d=\r\n", $reg );
  my $rs = $self->sendCmd( $s, ':r' );
  $rs =~ s/^:r\d\d=//x;
  return $rs;
}

sub OnOff
{
  my ($self,$o1,$o2) = @_;
  $o1  = $o1 ? '1' : '0';
  $o2  = $o2 ? '1' : '0';

  my $s   = $o1 . ',' . $o2;

  return $self->setReg( 20, $s );
}

sub setWave
{
  my ($self, $tp, $ch ) = @_;
  if( !defined( $tp ) ) {
    return '';
  }
  $tp   = 0 + $tp;
  my $reg  = $ch ? 22 : 21;

  return $self->setReg( $reg, '' . $tp );
}


sub setFreq
{
  my ($self, $freq, $ch ) = @_;
  if( !defined( $freq ) ) {
    return '';
  }
  $freq = 0.0 + $freq;
  my $reg  = $ch ? 24 : 23;
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
  my ($self, $ch ) = @_;
  my $reg  = $ch ? 24 : 23;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    return 0.0;
  }

  my $fs   = 0;
  my $r    = 0;

  if( $s =~ /^(\d+),(\d)/x  ) {
    $fs   = 0 + $1;
    $r    = 0 + $2;
  } else {
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

sub setFreqCheck
{
  my ($self, $f, $ch ) = @_;

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
  print( STDERR "?2 f= $f f_in= $f_in \n" );
  usleep( $self->{wait_after_bad_set} );
  }

  return 0;
}

sub setVpp
{
  my ($self, $v, $ch ) = @_;
  if( !defined( $v ) ) {
    return '';
  }
  my $vi    = 1000.0 * (0.0 + $v);
  my $reg  = $ch ? 26 : 25;

  my $s = sprintf( "%d", int($vi) );

  return $self->setReg( $reg, $s );
}


sub setBias
{
  my ($self, $bias, $ch ) = @_;
  if( !defined( $bias ) ) {
    return '';
  }
  my $bi   = 1000 + 100.0 * (0.0 + $bias);
  my $reg  = $ch ? 28 : 27;

  my $s = sprintf( "%d", int($bi) );

  return $self->setReg( $reg, $s );
}


sub setDuty
{
  my ($self, $du, $ch ) = @_;
  if( !defined( $du ) ) {
    return '';
  }
  my $dui   = 1000.0 * (0.0 + $du);
  my $reg  = $ch ? 30 : 29;

  my $s = sprintf( "%d", int($dui) );

  return $self->setReg( $reg, $s );
}

sub getDuty
{
  my ($self, $ch ) = @_;
  my $reg  = $ch ? 30 : 29;
  my $s = $self->getReg( $reg );
  if( !$s ) {
    return 0.0;
  }

  if( $s =~ /^(\d+)\./x ) {
    return ( 0.001 * $1 );
  }

  return 0.0;
}

sub setDutyCheck
{
  my ($self, $du, $ch ) = @_;

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
  print( STDERR "?2 du= $du du_in = $du_in\n" );
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
