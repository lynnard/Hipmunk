{-# LANGUAGE FunctionalDependencies #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE TypeSynonymInstances #-}
{-# CFILES
      Chipmunk-5.3.4/src/chipmunk.c,
      Chipmunk-5.3.4/src/constraints/cpConstraint.c,
      Chipmunk-5.3.4/src/constraints/cpDampedRotarySpring.c,
      Chipmunk-5.3.4/src/constraints/cpDampedSpring.c,
      Chipmunk-5.3.4/src/constraints/cpGearJoint.c,
      Chipmunk-5.3.4/src/constraints/cpGrooveJoint.c,
      Chipmunk-5.3.4/src/constraints/cpPinJoint.c,
      Chipmunk-5.3.4/src/constraints/cpPivotJoint.c,
      Chipmunk-5.3.4/src/constraints/cpRatchetJoint.c,
      Chipmunk-5.3.4/src/constraints/cpRotaryLimitJoint.c,
      Chipmunk-5.3.4/src/constraints/cpSimpleMotor.c,
      Chipmunk-5.3.4/src/constraints/cpSlideJoint.c,
      Chipmunk-5.3.4/src/cpArbiter.c,
      Chipmunk-5.3.4/src/cpArray.c,
      Chipmunk-5.3.4/src/cpBB.c,
      Chipmunk-5.3.4/src/cpBody.c,
      Chipmunk-5.3.4/src/cpCollision.c,
      Chipmunk-5.3.4/src/cpHashSet.c,
      Chipmunk-5.3.4/src/cpPolyShape.c,
      Chipmunk-5.3.4/src/cpShape.c,
      Chipmunk-5.3.4/src/cpSpace.c,
      Chipmunk-5.3.4/src/cpSpaceComponent.c,
      Chipmunk-5.3.4/src/cpSpaceHash.c,
      Chipmunk-5.3.4/src/cpSpaceQuery.c,
      Chipmunk-5.3.4/src/cpSpaceStep.c,
      Chipmunk-5.3.4/src/cpVect.c,
      Physics/Hipmunk/wrapper.c #-}

-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Common.hsc
-- Copyright   :  (c) 2008-2010 Felipe A. Lessa
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  provisional
-- Portability :  portable (needs FFI)
--
-- Functionality used by various modules and routines for
-- initialization and change of global variables.
--
-----------------------------------------------------------------------------

module Physics.Hipmunk.Common
    (-- * Initialization
     initChipmunk,

     -- * Basic data types
     CpFloat,
     infinity,
     Time,
     Angle,
     Distance,
     Damping,

     -- * Shape
     Mass,
     ShapeType(Circle, LineSegment, Polygon),
     circleRadius,
     lineStart,
     lineEnd,
     lineThickness,
     polyVertices,
     HasShapeDefinition(..),
     ShapeDefinition(ShapeDefinition),
     ShapeDefinition',

     -- * Global variables
     -- $global_vars

     -- ** Shape counter
     -- $shape_counter
     resetShapeCounter,

     -- ** Contact persistence
     -- $contact_persistence
     --contactPersistence,

     -- ** Collision slop
     -- $collision_slop
     --collisionSlop,

     -- ** Bias coefficient
     -- $bias_coef
     BiasCoef,
     --biasCoef,

     -- ** Constraint bias coefficient
     -- $constraint_bias_coef
     --constraintBiasCoef,

     -- * Vectors
     Vector,
     Vector',
     Position,
     Position',
    )
    where

import Data.Default
import Foreign hiding (rotate)
import Linear
import Linear.Affine (Point(..))
import Control.Lens
-- import Foreign.C.Types (CInt)
#include "wrapper.h"

-- error' :: String -> a
-- error' = error . ("Physics.Hipmunk.Common: " ++)

-- | Initilizes the Chipmunk library. This should be called
--   once before using any functions of this library.
initChipmunk :: IO ()
initChipmunk = cpInitChipmunk

foreign import ccall unsafe "wrapper.h"
    cpInitChipmunk :: IO ()


-- | The floating point type used internally in Chipmunk.
type CpFloat = #{type cpFloat}

-- | @infinity@ may be used to create bodies with
--   an infinite mass.
infinity :: CpFloat
infinity = 1e1000

-- | Type synonym used to hint that the argument or result
--   represents time.
type Time = CpFloat

-- | Type synonym used to hint that the argument or result
--   represents an angle in radians.
type Angle = CpFloat

-- | Type synonym used to hint that the argument or result
--   represents a distance.
type Distance = CpFloat

-- | Type synonym used to hint that the argument or result
--   represents a damping constant.
type Damping = CpFloat

-- $global_vars
--   Chipmunk tries to maintain a very few number of global
--   variables to allow multiple @Space@s to be used
--   simultaneously, however there are some.

-- $shape_counter
--   The shape counter is a global counter used for creating
--   unique hash identifiers to the shapes.

-- | @resetShapeCounter@ reset the shape counter to its default
--   value.  This is used to add determinism to a simulation.  As
--   the ids created with this counter may affect the order in
--   which the collisions happen, there may be very slight
--   differences in different simulations.  It may be very useful
--   to call @resetShapeCounter@ everytime you start a new
--   simulation.
--
--   However, be careful as you should not use shapes created
--   before a call to @resetCounter@ with shapes created after it
--   as they may have the same id.  This means that you can't add
--   shapes created after the call to a space created before it.
resetShapeCounter :: IO ()
resetShapeCounter = cpResetShapeIdCounter

foreign import ccall unsafe "wrapper.h"
    cpResetShapeIdCounter :: IO ()

-- makeStateVarFromPtr :: Storable a => Ptr a -> StateVar a
-- makeStateVarFromPtr p = makeStateVar (peek p) (poke p)

type BiasCoef = CpFloat

-- | A two-dimensional vector. It is an instance of 'Num'
--   however the operations 'signum' and @(*)@ are not
--   supported.
type Vector a = V2 a
type Vector' = Vector CpFloat

-- | Type synonym used to hint that the argument or result
--   represents a position.
type Position a = Point V2 a
type Position' = Position CpFloat


instance {-# OVERLAPPING #-} Storable Vector' where
    sizeOf _    = #{size cpVect}
    alignment _ = alignment (undefined :: CpFloat)
    peek ptr = do
      x <- #{peek cpVect, x} ptr
      y <- #{peek cpVect, y} ptr
      return (V2 x y)
    poke ptr (V2 x y) = do
      #{poke cpVect, x} ptr x
      #{poke cpVect, y} ptr y

type Mass = CpFloat

-- | There are three types of shapes that can be attached
--   to bodies:
data ShapeType a =
    -- | A circle is the fastest collision type. It also
    --   rolls smoothly.
    Circle {_circleRadius :: !a}

    -- | A line segment is meant to be used as a static
    --   shape. (It can be used with moving bodies, however
    --   two line segments never generate collisions between
    --   each other.)
  | LineSegment {_lineStart     :: !(Position a),
                 _lineEnd       :: !(Position a),
                 _lineThickness :: !a}

    -- | Polygons are the slowest of all shapes but
    --   the most flexible. The list of vertices must form
    --   a convex hull with clockwise winding.
    --   Note that if you want a non-convex polygon you may
    --   add several convex polygons to the body.
  | Polygon {_polyVertices :: ![Position a]}
    deriving (Eq, Ord, Show)

makeLenses ''ShapeType

instance Functor ShapeType where
    fmap f (Circle a) = Circle (f a)
    fmap f (LineSegment start end thick) = LineSegment (f <$> start) (f <$> end) (f thick)
    fmap f (Polygon verts) = Polygon (fmap f <$> verts)

-- TODO: parameterize over double or float?
data ShapeDefinition a = ShapeDefinition
    { _shapeType :: !(ShapeType a)
    , _shapeOffset :: !(Position a)
    , _shapeMass :: !a
    , _shapeCategoryMask :: !Word64
    , _shapeCollisionMask :: !Word64
    } deriving (Eq, Ord, Show)

makeClassy ''ShapeDefinition

type ShapeDefinition' = ShapeDefinition CpFloat

instance (Num a, Fractional a) => Default (ShapeDefinition a) where
    def = ShapeDefinition
        { _shapeType = LineSegment 0 0 0
        , _shapeOffset = 0
        , _shapeMass = 1/0
        , _shapeCategoryMask = 0
        -- mask that allows it to collide with everything else
        , _shapeCollisionMask = maxBound
        }
