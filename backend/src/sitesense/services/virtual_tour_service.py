import io
import hashlib
import json
import uuid
from dataclasses import asdict
from datetime import datetime, timezone

from PIL import Image, UnidentifiedImageError

from sitesense.core.exceptions import ConflictError, NotFoundError
from sitesense.db.models.virtual_tour import CaptureStatus, VirtualTourCapture
from sitesense.domain.entities import CaptureMetadata, StitchManifest, StitchedInput
from sitesense.repositories.virtual_tour_repository import VirtualTourRepository
from sitesense.services.capture_storage import LocalCaptureStorage


class VirtualTourService:
    def __init__(
        self,
        repository: VirtualTourRepository,
        storage: LocalCaptureStorage,
        *,
        max_capture_bytes: int,
        max_capture_count: int,
    ) -> None:
        self._repository = repository
        self._storage = storage
        self._max_capture_bytes = max_capture_bytes
        self._max_capture_count = max_capture_count

    async def create_upload_session(
        self, *, property_id: uuid.UUID, owner_id: uuid.UUID, expected_capture_count: int
    ):
        if not 1 <= expected_capture_count <= self._max_capture_count:
            raise ConflictError(f"expected_capture_count must be 1–{self._max_capture_count}")
        return await self._repository.create(
            property_id=property_id,
            owner_id=owner_id,
            expected_capture_count=expected_capture_count,
        )

    async def upload_capture(
        self,
        *,
        tour_id: uuid.UUID,
        owner_id: uuid.UUID,
        metadata: CaptureMetadata,
        image: bytes,
        content_type: str,
    ) -> tuple[VirtualTourCapture, bool]:
        tour = await self._require_uploading_tour(tour_id, owner_id)
        if not 1 <= metadata.target_index <= tour.expected_capture_count:
            raise ConflictError("target_index is outside this tour's capture plan")
        if not image or len(image) > self._max_capture_bytes:
            raise ConflictError("image is empty or exceeds the configured capture limit")
        extension, width, height = self._validate_image(image, content_type)
        digest = hashlib.sha256(image).hexdigest()
        existing = await self._repository.get_capture(tour_id, metadata.target_index)
        if existing is not None:
            if existing.sha256 == digest:
                return existing, True
            raise ConflictError("a different image already exists for this target_index")
        stored = await self._storage.put_capture(
            tour_id=tour_id,
            target_index=metadata.target_index,
            content=image,
            extension=extension,
        )
        capture = VirtualTourCapture(
            id=uuid.uuid4(),
            tour_id=tour_id,
            target_index=metadata.target_index,
            storage_key=stored.key,
            sha256=stored.sha256,
            content_type=content_type,
            byte_size=stored.byte_size,
            width=width,
            height=height,
            pose=metadata.pose,
            captured_at=metadata.captured_at,
        )
        return await self._repository.add_capture(capture), False

    async def finalise_for_stitching(self, *, tour_id: uuid.UUID, owner_id: uuid.UUID):
        tour = await self._repository.get_for_owner(tour_id, owner_id)
        if tour is None:
            raise NotFoundError("virtual tour was not found")
        if tour.status is not CaptureStatus.uploading:
            raise ConflictError(f"tour is already {tour.status.value}")
        if len(tour.captures) != tour.expected_capture_count:
            raise ConflictError(
                f"all {tour.expected_capture_count} captures are required before stitching"
            )
        manifest = StitchManifest(
            tour_id=tour.id,
            property_id=tour.property_id,
            captures=tuple(
                StitchedInput(
                    target_index=capture.target_index,
                    storage_key=capture.storage_key,
                    sha256=capture.sha256,
                    pose=capture.pose,
                )
                for capture in tour.captures
            ),
        )
        content = json.dumps(
            {
                "schema_version": 1,
                "tour_id": str(manifest.tour_id),
                "property_id": str(manifest.property_id),
                "created_at": datetime.now(timezone.utc).isoformat(),
                "captures": [asdict(capture) for capture in manifest.captures],
            },
            sort_keys=True,
            default=str,
        ).encode()
        key = await self._storage.put_manifest(tour_id=tour.id, content=content)
        return await self._repository.mark_stitch_pending(tour, key)

    async def get_tour(self, *, tour_id: uuid.UUID, owner_id: uuid.UUID):
        tour = await self._repository.get_for_owner(tour_id, owner_id)
        if tour is None:
            raise NotFoundError("virtual tour was not found")
        return tour

    async def _require_uploading_tour(self, tour_id: uuid.UUID, owner_id: uuid.UUID):
        tour = await self.get_tour(tour_id=tour_id, owner_id=owner_id)
        if tour.status is not CaptureStatus.uploading:
            raise ConflictError(f"tour cannot accept images while {tour.status.value}")
        return tour

    @staticmethod
    def _validate_image(image: bytes, content_type: str) -> tuple[str, int, int]:
        allowed = {"image/jpeg": "jpg", "image/png": "png"}
        extension = allowed.get(content_type.lower())
        if extension is None:
            raise ConflictError("only image/jpeg and image/png captures are accepted")
        try:
            with Image.open(io.BytesIO(image)) as decoded:
                decoded.verify()
            with Image.open(io.BytesIO(image)) as decoded:
                width, height = decoded.size
        except (UnidentifiedImageError, OSError) as error:
            raise ConflictError("uploaded capture is not a valid image") from error
        # CPU frames may be encoded in portrait orientation. Preserve the
        # 640×480 minimum quality threshold without incorrectly rejecting a
        # rotated 480×640 capture.
        if max(width, height) < 640 or min(width, height) < 480:
            raise ConflictError("capture resolution is too small for stitching")
        return extension, width, height
